// This is an advanced implementation of the algorithm described in the
// following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

// Modifier: Livox               dev@livoxtech.com

// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
#include <omp.h>
#include <algorithm>
#include <atomic>
#include <limits>
#include <mutex>
#include <math.h>
#include <thread>
#include <fstream>
#include <csignal>
#include <chrono>
#include <unistd.h>
#include <Python.h>
#include <so3_math.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "IMU_Processing.hpp"
#include "frontend_guard.hpp"
#include "stationary_guard.hpp"
#include "stationary_update.hpp"
#include <unitree_go/msg/sport_mode_state.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include "preprocess.h"
#include <ikd-Tree/ikd_Tree.h>

#define INIT_TIME           (0.1)
#define LASER_POINT_COV     (0.001)
#define MAXN                (720000)
#define PUBFRAME_PERIOD     (20)

/*** Time Log Variables ***/
double kdtree_incremental_time = 0.0, kdtree_search_time = 0.0, kdtree_delete_time = 0.0;
double T1[MAXN], s_plot[MAXN], s_plot2[MAXN], s_plot3[MAXN], s_plot4[MAXN], s_plot5[MAXN], s_plot6[MAXN], s_plot7[MAXN], s_plot8[MAXN], s_plot9[MAXN], s_plot10[MAXN], s_plot11[MAXN];
double match_time = 0, solve_time = 0, solve_const_H_time = 0;
int    kdtree_size_st = 0, kdtree_size_end = 0, add_point_size = 0, kdtree_delete_counter = 0;
bool   runtime_pos_log = false, pcd_save_en = false, time_sync_en = false, extrinsic_est_en = true, path_en = true;
/**************************/

float res_last[100000] = {0.0};
float DET_RANGE = 300.0f;
const float MOV_THRESHOLD = 1.5f;
double time_diff_lidar_to_imu = 0.0;

mutex mtx_buffer;
condition_variable sig_buffer;

string root_dir = ROOT_DIR;
string map_file_path, lid_topic, imu_topic;
string world_frame = "camera_init";
string body_frame = "body";

double res_mean_last = 0.05, total_residual = 0.0;
double last_timestamp_lidar = 0, last_timestamp_imu = -1.0;
double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;
double filter_size_corner_min = 0, filter_size_surf_min = 0, filter_size_map_min = 0, fov_deg = 0;
double cube_len = 0, HALF_FOV_COS = 0, FOV_DEG = 0, total_distance = 0, lidar_end_time = 0, first_lidar_time = 0.0;
int    effct_feat_num = 0, time_log_counter = 0, scan_count = 0;
int    iterCount = 0, feats_down_size = 0, NUM_MAX_ITERATIONS = 0, laserCloudValidNum = 0, pcd_save_interval = -1, pcd_index = 0;
bool   point_selected_surf[100000] = {0};
bool   lidar_pushed, flg_first_scan = true, flg_exit = false, flg_EKF_inited;
bool   scan_pub_en = false, dense_pub_en = false, scan_body_pub_en = false;
bool   tf_pub_en = true;
bool    is_first_lidar = true;
double body_publish_voxel_size = 0.08;
double body_publish_max_range = 0.0;

vector<vector<int>>  pointSearchInd_surf; 
vector<BoxPointType> cub_needrm;
vector<PointVector>  Nearest_Points; 
vector<double>       extrinT(3, 0.0);
vector<double>       extrinR(9, 0.0);
deque<double>                     time_buffer;
deque<PointCloudXYZI::Ptr>        lidar_buffer;
deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu_buffer;

PointCloudXYZI::Ptr featsFromMap(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_body(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_world(new PointCloudXYZI());
PointCloudXYZI::Ptr normvec(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr laserCloudOri(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr corr_normvect(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr _featsArray;

pcl::VoxelGrid<PointType> downSizeFilterSurf;
pcl::VoxelGrid<PointType> downSizeFilterMap;

KD_TREE<PointType> ikdtree;

V3F XAxisPoint_body(LIDAR_SP_LEN, 0.0, 0.0);
V3F XAxisPoint_world(LIDAR_SP_LEN, 0.0, 0.0);
V3D euler_cur;
V3D position_last(Zero3d);
V3D Lidar_T_wrt_IMU(Zero3d);
M3D Lidar_R_wrt_IMU(Eye3d);

/*** EKF inputs and output ***/
MeasureGroup Measures;
esekfom::esekf<state_ikfom, 12, input_ikfom> kf;
state_ikfom state_point;
vect3 pos_lid;

nav_msgs::msg::Path path;
nav_msgs::msg::Odometry odomAftMapped;
geometry_msgs::msg::Quaternion geoQuat;
geometry_msgs::msg::PoseStamped msg_body_pose;

shared_ptr<Preprocess> p_pre(new Preprocess());
shared_ptr<ImuProcess> p_imu(new ImuProcess());

void SigHandle(int sig)
{
    flg_exit = true;
    std::cout << "catch sig %d" << sig << std::endl;
    sig_buffer.notify_all();
    rclcpp::shutdown();
}

inline void dump_lio_state_to_log(FILE *fp)  
{
    V3D rot_ang(Log(state_point.rot.toRotationMatrix()));
    fprintf(fp, "%lf ", Measures.lidar_beg_time - first_lidar_time);
    fprintf(fp, "%lf %lf %lf ", rot_ang(0), rot_ang(1), rot_ang(2));                   // Angle
    fprintf(fp, "%lf %lf %lf ", state_point.pos(0), state_point.pos(1), state_point.pos(2)); // Pos  
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // omega  
    fprintf(fp, "%lf %lf %lf ", state_point.vel(0), state_point.vel(1), state_point.vel(2)); // Vel  
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // Acc  
    fprintf(fp, "%lf %lf %lf ", state_point.bg(0), state_point.bg(1), state_point.bg(2));    // Bias_g  
    fprintf(fp, "%lf %lf %lf ", state_point.ba(0), state_point.ba(1), state_point.ba(2));    // Bias_a  
    fprintf(fp, "%lf %lf %lf ", state_point.grav[0], state_point.grav[1], state_point.grav[2]); // Bias_a  
    fprintf(fp, "\r\n");  
    fflush(fp);
}

void pointBodyToWorld_ikfom(PointType const * const pi, PointType * const po, state_ikfom &s)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}


void pointBodyToWorld(PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

template<typename T>
void pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po)
{
    V3D p_body(pi[0], pi[1], pi[2]);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po[0] = p_global(0);
    po[1] = p_global(1);
    po[2] = p_global(2);
}

void RGBpointBodyToWorld(PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

void RGBpointBodyLidarToIMU(PointType const * const pi, PointType * const po)
{
    V3D p_body_lidar(pi->x, pi->y, pi->z);
    V3D p_body_imu(state_point.offset_R_L_I*p_body_lidar + state_point.offset_T_L_I);

    po->x = p_body_imu(0);
    po->y = p_body_imu(1);
    po->z = p_body_imu(2);
    po->intensity = pi->intensity;
}

void points_cache_collect()
{
    PointVector points_history;
    ikdtree.acquire_removed_points(points_history);
    // for (int i = 0; i < points_history.size(); i++) _featsArray->push_back(points_history[i]);
}

BoxPointType LocalMap_Points;
bool Localmap_Initialized = false;
void lasermap_fov_segment()
{
    cub_needrm.clear();
    kdtree_delete_counter = 0;
    kdtree_delete_time = 0.0;    
    pointBodyToWorld(XAxisPoint_body, XAxisPoint_world);
    V3D pos_LiD = pos_lid;
    if (!Localmap_Initialized){
        for (int i = 0; i < 3; i++){
            LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
            LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
        }
        Localmap_Initialized = true;
        return;
    }
    float dist_to_map_edge[3][2];
    bool need_move = false;
    for (int i = 0; i < 3; i++){
        dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE || dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE) need_move = true;
    }
    if (!need_move) return;
    BoxPointType New_LocalMap_Points, tmp_boxpoints;
    New_LocalMap_Points = LocalMap_Points;
    float mov_dist = max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9, double(DET_RANGE * (MOV_THRESHOLD -1)));
    for (int i = 0; i < 3; i++){
        tmp_boxpoints = LocalMap_Points;
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] -= mov_dist;
            New_LocalMap_Points.vertex_min[i] -= mov_dist;
            tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        } else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] += mov_dist;
            New_LocalMap_Points.vertex_min[i] += mov_dist;
            tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        }
    }
    LocalMap_Points = New_LocalMap_Points;

    points_cache_collect();
    double delete_begin = omp_get_wtime();
    if(cub_needrm.size() > 0) kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm);
    kdtree_delete_time = omp_get_wtime() - delete_begin;
}

void standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::UniquePtr msg) 
{
    double cur_time = get_time_sec(msg->header.stamp);
    double preprocess_start_time = omp_get_wtime();
    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);

    {
        std::lock_guard<std::mutex> lock(mtx_buffer);
        scan_count ++;
        if (!is_first_lidar && cur_time < last_timestamp_lidar)
        {
            std::cerr << "lidar loop back, clear buffer" << std::endl;
            lidar_buffer.clear();
            time_buffer.clear();
            lidar_pushed = false;
        }
        is_first_lidar = false;
        lidar_buffer.push_back(ptr);
        time_buffer.push_back(cur_time);
        last_timestamp_lidar = cur_time;
        if (scan_count < MAXN)
        {
            s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
        }
    }
    sig_buffer.notify_all();
}

double timediff_lidar_wrt_imu = 0.0;
bool   timediff_set_flg = false;
void livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::UniquePtr msg) 
{
    double cur_time = get_time_sec(msg->header.stamp);
    double preprocess_start_time = omp_get_wtime();
    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);

    {
        std::lock_guard<std::mutex> lock(mtx_buffer);
        scan_count ++;
        if (!is_first_lidar && cur_time < last_timestamp_lidar)
        {
            std::cerr << "lidar loop back, clear buffer" << std::endl;
            lidar_buffer.clear();
            time_buffer.clear();
            lidar_pushed = false;
        }
        is_first_lidar = false;
        last_timestamp_lidar = cur_time;

        if (!time_sync_en && abs(last_timestamp_imu - last_timestamp_lidar) > 10.0 && !imu_buffer.empty())
        {
            printf("IMU and LiDAR not Synced, IMU time: %lf, lidar header time: %lf \n",last_timestamp_imu, last_timestamp_lidar);
        }

        if (time_sync_en && !timediff_set_flg && abs(last_timestamp_lidar - last_timestamp_imu) > 1 && !imu_buffer.empty())
        {
            timediff_set_flg = true;
            timediff_lidar_wrt_imu = last_timestamp_lidar + 0.1 - last_timestamp_imu;
            printf("Self sync IMU and LiDAR, time diff is %.10lf \n", timediff_lidar_wrt_imu);
        }

        lidar_buffer.push_back(ptr);
        time_buffer.push_back(last_timestamp_lidar);
        if (scan_count < MAXN)
        {
            s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
        }
    }
    sig_buffer.notify_all();
}

void imu_cbk(const sensor_msgs::msg::Imu::UniquePtr msg_in)
{
    // cout<<"IMU got at: "<<msg_in->header.stamp.toSec()<<endl;
    sensor_msgs::msg::Imu::SharedPtr msg(new sensor_msgs::msg::Imu(*msg_in));
    

    msg->header.stamp = get_ros_time(get_time_sec(msg_in->header.stamp) - time_diff_lidar_to_imu);
    {
        std::lock_guard<std::mutex> lock(mtx_buffer);
        if (abs(timediff_lidar_wrt_imu) > 0.1 && time_sync_en)
        {
            msg->header.stamp =
                rclcpp::Time(timediff_lidar_wrt_imu + get_time_sec(msg_in->header.stamp));
        }

        double timestamp = get_time_sec(msg->header.stamp);
        if (timestamp < last_timestamp_imu)
        {
            std::cerr << "imu loop back, clear buffer" << std::endl;
            imu_buffer.clear();
        }

        last_timestamp_imu = timestamp;
        imu_buffer.push_back(msg);
    }
    sig_buffer.notify_all();
}

double lidar_mean_scantime = 0.0;
int    scan_num = 0;
bool sync_packages(MeasureGroup &meas)
{
    std::lock_guard<std::mutex> lock(mtx_buffer);
    if (lidar_buffer.empty() || imu_buffer.empty()) {
        return false;
    }

    // Building the initial tree (or a slow scan-to-map update) can queue
    // several clouds while the independent sensor callbacks keep running.
    // Processing that FIFO makes healthy live input fail the age guard.
    // Catch up to the newest scan already covered by IMU, keeping ALL IMU
    // samples since the previous processed scan for continuous propagation.
    const auto skipped = fast_lio_guard::latest_covered_scan(
        lidar_buffer.size(), last_timestamp_imu, [](std::size_t i) {
            const auto &cloud = lidar_buffer[i];
            double duration = lidar_mean_scantime;
            if (cloud->size() > 1) {
                duration = std::max(
                    duration, cloud->points.back().curvature / 1000.0);
            }
            if (p_pre->lidar_type == HAP_POINTCLOUD2) {
                duration = cloud->empty() ? 0.0 : cloud->points.back().curvature / 1000.0;
            }
            return time_buffer[i] + duration;
        });
    for (std::size_t i = 0; i < skipped; ++i) {
        lidar_buffer.pop_front();
        time_buffer.pop_front();
    }
    if (skipped > 0) {
        // Measures may refer to a front scan that was waiting for its IMU.
        lidar_pushed = false;
    }

    /*** push a lidar scan ***/
    if(!lidar_pushed)
    {
        meas.lidar = lidar_buffer.front();
        meas.lidar_beg_time = time_buffer.front();
        if (p_pre->lidar_type == HAP_POINTCLOUD2)
        {
            lidar_end_time = meas.lidar_beg_time + fast_lio_guard::scan_duration(
                meas.lidar->empty() ? 0.0 : meas.lidar->points.back().curvature / 1000.0,
                lidar_mean_scantime, true);
        }
        else if (meas.lidar->points.size() <= 1) // time too little
        {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
            std::cerr << "Too few input point cloud!\n";
        }
        else if (meas.lidar->points.back().curvature / double(1000) < 0.5 * lidar_mean_scantime)
        {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
        }
        else
        {
            scan_num ++;
            lidar_end_time = meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000);
            lidar_mean_scantime += (meas.lidar->points.back().curvature / double(1000) - lidar_mean_scantime) / scan_num;
        }

        meas.lidar_end_time = lidar_end_time;

        lidar_pushed = true;
    }

    if (last_timestamp_imu < lidar_end_time)
    {
        return false;
    }

    /*** push imu data, and pop from imu buffer ***/
    double imu_time = get_time_sec(imu_buffer.front()->header.stamp);
    meas.imu.clear();
    while ((!imu_buffer.empty()) && (imu_time < lidar_end_time))
    {
        imu_time = get_time_sec(imu_buffer.front()->header.stamp);
        if(imu_time > lidar_end_time) break;
        meas.imu.push_back(imu_buffer.front());
        imu_buffer.pop_front();
    }

    lidar_buffer.pop_front();
    time_buffer.pop_front();
    lidar_pushed = false;
    return true;
}

int process_increments = 0;
void map_incremental()
{
    PointVector PointToAdd;
    PointVector PointNoNeedDownsample;
    PointToAdd.reserve(feats_down_size);
    PointNoNeedDownsample.reserve(feats_down_size);
    for (int i = 0; i < feats_down_size; i++)
    {
        /* transform to world frame */
        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
        /* decide if need add to map */
        if (!Nearest_Points[i].empty() && flg_EKF_inited)
        {
            const PointVector &points_near = Nearest_Points[i];
            bool need_add = true;
            BoxPointType Box_of_Point;
            PointType downsample_result, mid_point; 
            mid_point.x = floor(feats_down_world->points[i].x/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.y = floor(feats_down_world->points[i].y/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.z = floor(feats_down_world->points[i].z/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            float dist  = calc_dist(feats_down_world->points[i],mid_point);
            if (fabs(points_near[0].x - mid_point.x) > 0.5 * filter_size_map_min && fabs(points_near[0].y - mid_point.y) > 0.5 * filter_size_map_min && fabs(points_near[0].z - mid_point.z) > 0.5 * filter_size_map_min){
                PointNoNeedDownsample.push_back(feats_down_world->points[i]);
                continue;
            }
            for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i ++)
            {
                if (points_near.size() < NUM_MATCH_POINTS) break;
                if (calc_dist(points_near[readd_i], mid_point) < dist)
                {
                    need_add = false;
                    break;
                }
            }
            if (need_add) PointToAdd.push_back(feats_down_world->points[i]);
        }
        else
        {
            PointToAdd.push_back(feats_down_world->points[i]);
        }
    }

    double st_time = omp_get_wtime();
    add_point_size = ikdtree.Add_Points(PointToAdd, true);
    ikdtree.Add_Points(PointNoNeedDownsample, false); 
    add_point_size = PointToAdd.size() + PointNoNeedDownsample.size();
    kdtree_incremental_time = omp_get_wtime() - st_time;
}

PointCloudXYZI::Ptr pcl_wait_pub(new PointCloudXYZI());
PointCloudXYZI::Ptr pcl_wait_save(new PointCloudXYZI());
void publish_frame_world(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull)
{
    if(scan_pub_en)
    {
        PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en ? feats_undistort : feats_down_body);
        int size = laserCloudFullRes->points.size();
        PointCloudXYZI::Ptr laserCloudWorld( \
                        new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++)
        {
            RGBpointBodyToWorld(&laserCloudFullRes->points[i], \
                                &laserCloudWorld->points[i]);
        }

        sensor_msgs::msg::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
        // laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
        laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
        laserCloudmsg.header.frame_id = world_frame;
        pubLaserCloudFull->publish(laserCloudmsg);
    }

    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. noted that pcd save will influence the real-time performences **/
    /*
    if (pcd_save_en)
    {
        int size = feats_undistort->points.size();
        PointCloudXYZI::Ptr laserCloudWorld( \
                        new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++)
        {
            RGBpointBodyToWorld(&feats_undistort->points[i], \
                                &laserCloudWorld->points[i]);
        }
        *pcl_wait_save += *laserCloudWorld;

        static int scan_wait_num = 0;
        scan_wait_num ++;
        if (pcl_wait_save->size() > 0 && pcd_save_interval > 0  && scan_wait_num >= pcd_save_interval)
        {
            pcd_index ++;
            string all_points_dir(string(string(ROOT_DIR) + "PCD/scans_") + to_string(pcd_index) + string(".pcd"));
            pcl::PCDWriter pcd_writer;
            cout << "current scan saved to /PCD/" << all_points_dir << endl;
            pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
            pcl_wait_save->clear();
            scan_wait_num = 0;
        }
    }
    */
}

void publish_frame_body(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull_body)
{
    PointCloudXYZI::Ptr body_source = feats_undistort;
    PointCloudXYZI::Ptr body_range_filtered(new PointCloudXYZI());
    PointCloudXYZI::Ptr body_filtered(new PointCloudXYZI());

    // /cloud_registered_body feeds loop closure and the short-range obstacle
    // pipeline, not the scan-to-map estimator.  Do not pass very distant HAP
    // returns to PCL VoxelGrid: a 5 cm leaf over a large 3-D bounding box can
    // overflow PCL's 32-bit voxel index even when every point is finite.
    if (body_publish_max_range > 0.0)
    {
        const double maximum_squared_range =
            body_publish_max_range * body_publish_max_range;
        body_range_filtered->points.reserve(feats_undistort->size());
        for (const auto &point : feats_undistort->points)
        {
            const double squared_range =
                static_cast<double>(point.x) * point.x +
                static_cast<double>(point.y) * point.y +
                static_cast<double>(point.z) * point.z;
            if (squared_range <= maximum_squared_range)
            {
                body_range_filtered->points.push_back(point);
            }
        }
        body_range_filtered->width =
            static_cast<std::uint32_t>(body_range_filtered->points.size());
        body_range_filtered->height = 1;
        body_source = body_range_filtered;
    }

    if (!dense_pub_en && body_publish_voxel_size > 0.0)
    {
        pcl::VoxelGrid<PointType> body_voxel;
        const float leaf = static_cast<float>(body_publish_voxel_size);
        body_voxel.setLeafSize(leaf, leaf, leaf);
        body_voxel.setInputCloud(body_source);
        body_voxel.filter(*body_filtered);
        body_source = body_filtered;
    }

    int size = body_source->points.size();
    PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++)
    {
        RGBpointBodyLidarToIMU(&body_source->points[i], \
                            &laserCloudIMUBody->points[i]);
    }

    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
    laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
    laserCloudmsg.header.frame_id = body_frame;
    pubLaserCloudFull_body->publish(laserCloudmsg);
}

void publish_effect_world(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudEffect)
{
    PointCloudXYZI::Ptr laserCloudWorld( \
                    new PointCloudXYZI(effct_feat_num, 1));
    for (int i = 0; i < effct_feat_num; i++)
    {
        RGBpointBodyToWorld(&laserCloudOri->points[i], \
                            &laserCloudWorld->points[i]);
    }
    sensor_msgs::msg::PointCloud2 laserCloudFullRes3;
    pcl::toROSMsg(*laserCloudWorld, laserCloudFullRes3);
    laserCloudFullRes3.header.stamp = get_ros_time(lidar_end_time);
    laserCloudFullRes3.header.frame_id = world_frame;
    pubLaserCloudEffect->publish(laserCloudFullRes3);
}

void publish_map(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudMap)
{
    PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en ? feats_undistort : feats_down_body);
    int size = laserCloudFullRes->points.size();
    PointCloudXYZI::Ptr laserCloudWorld( \
                    new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++)
    {
        RGBpointBodyToWorld(&laserCloudFullRes->points[i], \
                            &laserCloudWorld->points[i]);
    }
    *pcl_wait_pub += *laserCloudWorld;

    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*pcl_wait_pub, laserCloudmsg);
    // laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
    laserCloudmsg.header.frame_id = world_frame;
    pubLaserCloudMap->publish(laserCloudmsg);

    // sensor_msgs::msg::PointCloud2 laserCloudMap;
    // pcl::toROSMsg(*featsFromMap, laserCloudMap);
    // laserCloudMap.header.stamp = get_ros_time(lidar_end_time);
    // laserCloudMap.header.frame_id = "camera_init";
    // pubLaserCloudMap->publish(laserCloudMap);
}

void save_to_pcd()
{
    pcl::PCDWriter pcd_writer;
    pcd_writer.writeBinary(map_file_path, *pcl_wait_pub);
}

template<typename T>
void set_posestamp(T & out)
{
    out.pose.position.x = state_point.pos(0);
    out.pose.position.y = state_point.pos(1);
    out.pose.position.z = state_point.pos(2);
    out.pose.orientation.x = geoQuat.x;
    out.pose.orientation.y = geoQuat.y;
    out.pose.orientation.z = geoQuat.z;
    out.pose.orientation.w = geoQuat.w;
    
}

void publish_odometry(const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped, std::unique_ptr<tf2_ros::TransformBroadcaster> & tf_br)
{
    odomAftMapped.header.frame_id = world_frame;
    odomAftMapped.child_frame_id = body_frame;
    odomAftMapped.header.stamp = get_ros_time(lidar_end_time);
    set_posestamp(odomAftMapped.pose);
    auto P = kf.get_P();
    for (int i = 0; i < 6; i ++)
    {
        int k = i < 3 ? i + 3 : i - 3;
        odomAftMapped.pose.covariance[i*6 + 0] = P(k, 3);
        odomAftMapped.pose.covariance[i*6 + 1] = P(k, 4);
        odomAftMapped.pose.covariance[i*6 + 2] = P(k, 5);
        odomAftMapped.pose.covariance[i*6 + 3] = P(k, 0);
        odomAftMapped.pose.covariance[i*6 + 4] = P(k, 1);
        odomAftMapped.pose.covariance[i*6 + 5] = P(k, 2);
    }
    pubOdomAftMapped->publish(odomAftMapped);

    if (!tf_pub_en)
    {
        return;
    }

    geometry_msgs::msg::TransformStamped trans;
    trans.header.frame_id = world_frame;
    trans.header.stamp = odomAftMapped.header.stamp;
    trans.child_frame_id = body_frame;
    trans.transform.translation.x = odomAftMapped.pose.pose.position.x;
    trans.transform.translation.y = odomAftMapped.pose.pose.position.y;
    trans.transform.translation.z = odomAftMapped.pose.pose.position.z;
    trans.transform.rotation.w = odomAftMapped.pose.pose.orientation.w;
    trans.transform.rotation.x = odomAftMapped.pose.pose.orientation.x;
    trans.transform.rotation.y = odomAftMapped.pose.pose.orientation.y;
    trans.transform.rotation.z = odomAftMapped.pose.pose.orientation.z;
    tf_br->sendTransform(trans);
}

void publish_path(rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath)
{
    set_posestamp(msg_body_pose);
    msg_body_pose.header.stamp = get_ros_time(lidar_end_time); // ros::Time().fromSec(lidar_end_time);
    msg_body_pose.header.frame_id = world_frame;

    /*** if path is too large, the rvis will crash ***/
    static int jjj = 0;
    jjj++;
    if (jjj % 10 == 0) 
    {
        path.poses.push_back(msg_body_pose);
        pubPath->publish(path);
    }
}

void h_share_model(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data)
{
    double match_start = omp_get_wtime();
    std::atomic<int> nearest_valid_count{0};
    std::atomic<int> plane_valid_count{0};
    // Compaction below writes by index. Capacity after clear() is not
    // constructed vector storage; keep every writable element in bounds.
    laserCloudOri->resize(feats_down_size);
    corr_normvect->resize(feats_down_size);
    total_residual = 0.0; 

    /** closest surface search and residual computation **/
    #ifdef MP_EN
        omp_set_num_threads(MP_PROC_NUM);
        #pragma omp parallel for
    #endif
    for (int i = 0; i < feats_down_size; i++)
    {
        PointType &point_body  = feats_down_body->points[i]; 
        PointType &point_world = feats_down_world->points[i]; 

        /* transform to world frame */
        V3D p_body(point_body.x, point_body.y, point_body.z);
        V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);
        point_world.x = p_global(0);
        point_world.y = p_global(1);
        point_world.z = p_global(2);
        point_world.intensity = point_body.intensity;

        vector<float> pointSearchSqDis(NUM_MATCH_POINTS);

        auto &points_near = Nearest_Points[i];

        if (ekfom_data.converge)
        {
            /** Find the closest surfaces in the map **/
            ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
            point_selected_surf[i] = points_near.size() < NUM_MATCH_POINTS ? false : pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false : true;
        }

        if (!point_selected_surf[i]) continue;
        nearest_valid_count.fetch_add(1, std::memory_order_relaxed);

        VF(4) pabcd;
        point_selected_surf[i] = false;
        if (esti_plane(pabcd, points_near, 0.1f))
        {
            plane_valid_count.fetch_add(1, std::memory_order_relaxed);
            float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3);
            float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());

            if (s > 0.9)
            {
                point_selected_surf[i] = true;
                normvec->points[i].x = pabcd(0);
                normvec->points[i].y = pabcd(1);
                normvec->points[i].z = pabcd(2);
                normvec->points[i].intensity = pd2;
                res_last[i] = abs(pd2);
            }
        }
    }
    
    effct_feat_num = 0;

    for (int i = 0; i < feats_down_size; i++)
    {
        if (point_selected_surf[i])
        {
            laserCloudOri->points[effct_feat_num] = feats_down_body->points[i];
            corr_normvect->points[effct_feat_num] = normvec->points[i];
            total_residual += res_last[i];
            effct_feat_num ++;
        }
    }

    laserCloudOri->resize(effct_feat_num);
    corr_normvect->resize(effct_feat_num);

    if (effct_feat_num < 1)
    {
        ekfom_data.valid = false;
        static std::atomic<uint64_t> invalid_call_count{0};
        const uint64_t call = invalid_call_count.fetch_add(1) + 1;
        if (call <= 20 || call % 100 == 0)
        {
            const V3D diagnostic_euler = SO3ToEuler(s.rot);
            std::cerr << "No Effective Points: call=" << call
                      << " scan=" << feats_down_size
                      << " map=" << ikdtree.validnum()
                      << " nearest=" << nearest_valid_count.load()
                      << " plane=" << plane_valid_count.load()
                      << " pos=" << s.pos.transpose()
                      << " rpy=" << diagnostic_euler.transpose()
                      << std::endl;
        }
        return;
    }

    res_mean_last = total_residual / effct_feat_num;
    match_time  += omp_get_wtime() - match_start;
    double solve_start_  = omp_get_wtime();
    
    /*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
    ekfom_data.h_x = MatrixXd::Zero(effct_feat_num, 12); //23
    ekfom_data.h.resize(effct_feat_num);

    for (int i = 0; i < effct_feat_num; i++)
    {
        const PointType &laser_p  = laserCloudOri->points[i];
        V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
        M3D point_be_crossmat;
        point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
        V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;
        M3D point_crossmat;
        point_crossmat<<SKEW_SYM_MATRX(point_this);

        /*** get the normal vector of closest surface/corner ***/
        const PointType &norm_p = corr_normvect->points[i];
        V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

        /*** calculate the Measuremnt Jacobian matrix H ***/
        V3D C(s.rot.conjugate() *norm_vec);
        V3D A(point_crossmat * C);
        if (extrinsic_est_en)
        {
            V3D B(point_be_crossmat * s.offset_R_L_I.conjugate() * C); //s.rot.conjugate()*norm_vec);
            ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
        }
        else
        {
            ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        }

        /*** Measuremnt: distance to the closest surface/corner ***/
        ekfom_data.h(i) = -norm_p.intensity;
    }
    solve_time += omp_get_wtime() - solve_start_;
}

class LaserMappingNode : public rclcpp::Node
{
public:
    LaserMappingNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : Node("laser_mapping", options)
    {
        this->declare_parameter<bool>("publish.path_en", true);
        this->declare_parameter<bool>("publish.effect_map_en", false);
        this->declare_parameter<bool>("publish.map_en", false);
        this->declare_parameter<bool>("publish.scan_publish_en", true);
        this->declare_parameter<bool>("publish.dense_publish_en", true);
        this->declare_parameter<bool>("publish.scan_bodyframe_pub_en", true);
        this->declare_parameter<double>("publish.body_voxel_size", 0.08);
        this->declare_parameter<bool>("publish.tf_en", true);
        this->declare_parameter<string>("publish.world_frame", "camera_init");
        this->declare_parameter<string>("publish.body_frame", "body");
        this->declare_parameter<int>("max_iteration", 4);
        this->declare_parameter<string>("map_file_path", "");
        this->declare_parameter<string>("common.lid_topic", "/livox/lidar");
        this->declare_parameter<string>("common.imu_topic", "/livox/imu");
        this->declare_parameter<bool>("common.time_sync_en", false);
        this->declare_parameter<double>("common.time_offset_lidar_to_imu", 0.0);
        this->declare_parameter<int>("common.imu_queue_depth", 2000);
        this->declare_parameter<int>("common.lidar_queue_depth", 50);
        this->declare_parameter<int>("common.executor_threads", 3);
        this->declare_parameter<double>("filter_size_corner", 0.5);
        this->declare_parameter<double>("filter_size_surf", 0.5);
        this->declare_parameter<double>("filter_size_map", 0.5);
        this->declare_parameter<double>("cube_side_length", 200.);
        this->declare_parameter<float>("mapping.det_range", 300.);
        this->declare_parameter<double>("mapping.fov_degree", 180.);
        this->declare_parameter<double>("mapping.gyr_cov", 0.1);
        this->declare_parameter<double>("mapping.acc_cov", 0.1);
        this->declare_parameter<double>("mapping.b_gyr_cov", 0.0001);
        this->declare_parameter<double>("mapping.b_acc_cov", 0.0001);
        this->declare_parameter<double>("preprocess.blind", 0.01);
        this->declare_parameter<int>("preprocess.lidar_type", AVIA);
        this->declare_parameter<int>("preprocess.scan_line", 16);
        this->declare_parameter<int>("preprocess.timestamp_unit", US);
        this->declare_parameter<int>("preprocess.scan_rate", 10);
        this->declare_parameter<int>("point_filter_num", 2);
        this->declare_parameter<bool>("feature_extract_enable", false);
        this->declare_parameter<bool>("runtime_pos_log_enable", false);
        this->declare_parameter<double>("publish.body_max_range", 0.0);
        this->declare_parameter<bool>("mapping.extrinsic_est_en", true);
        this->declare_parameter<double>("mapping.imu_init_duration", 5.0);
        this->declare_parameter<double>("mapping.initial_map_duration", 2.0);
        this->declare_parameter<int>("mapping.initial_map_min_voxels", 400);
        this->declare_parameter<double>("mapping.initial_map_max_duration", 10.0);
        this->declare_parameter<double>("mapping.input_max_age", 0.30);
        this->declare_parameter<double>("mapping.input_max_imu_gap", 0.05);
        this->declare_parameter<double>("mapping.input_max_scan_duration", 0.15);
        this->declare_parameter<double>("mapping.input_warmup_duration", 1.0);
        this->declare_parameter<bool>("mapping.stationary_constraint", false);
        this->declare_parameter<int>("mapping.quality_min_matches", 50);
        this->declare_parameter<double>("mapping.quality_min_match_ratio", 0.05);
        this->declare_parameter<double>("mapping.quality_max_residual", 0.15);
        this->declare_parameter<double>("mapping.safety_max_state_speed", 3.0);
        this->declare_parameter<double>("mapping.safety_max_pose_speed", 3.0);
        this->declare_parameter<double>("mapping.safety_max_angular_speed", 4.0);
        this->declare_parameter<double>("mapping.safety_max_position_radius", 100.0);
        this->declare_parameter<double>("mapping.safety_max_absolute_z", 5.0);
        this->declare_parameter<double>("mapping.safety_max_undistorted_range", 120.0);
        this->declare_parameter<double>("mapping.safety_max_invalid_point_fraction", 0.05);
        this->declare_parameter<int>("mapping.safety_corrupt_cloud_confirmation_samples", 3);
        this->declare_parameter<bool>("pcd_save.pcd_save_en", false);
        this->declare_parameter<int>("pcd_save.interval", -1);
        this->declare_parameter<vector<double>>("mapping.extrinsic_T", vector<double>());
        this->declare_parameter<vector<double>>("mapping.extrinsic_R", vector<double>());

        this->get_parameter_or<bool>("publish.path_en", path_en, true);
        this->get_parameter_or<bool>("publish.effect_map_en", effect_pub_en, false);
        this->get_parameter_or<bool>("publish.map_en", map_pub_en, false);
        this->get_parameter_or<bool>("publish.scan_publish_en", scan_pub_en, true);
        this->get_parameter_or<bool>("publish.dense_publish_en", dense_pub_en, true);
        this->get_parameter_or<bool>("publish.scan_bodyframe_pub_en", scan_body_pub_en, true);
        this->get_parameter_or<double>(
            "publish.body_voxel_size", body_publish_voxel_size, 0.08);
        this->get_parameter_or<double>(
            "publish.body_max_range", body_publish_max_range, 0.0);
        this->get_parameter_or<bool>("publish.tf_en", tf_pub_en, true);
        this->get_parameter_or<string>("publish.world_frame", world_frame, "camera_init");
        this->get_parameter_or<string>("publish.body_frame", body_frame, "body");
        this->get_parameter_or<int>("max_iteration", NUM_MAX_ITERATIONS, 4);
        this->get_parameter_or<string>("map_file_path", map_file_path, "");
        this->get_parameter_or<string>("common.lid_topic", lid_topic, "/livox/lidar");
        this->get_parameter_or<string>("common.imu_topic", imu_topic,"/livox/imu");
        this->get_parameter_or<bool>("common.time_sync_en", time_sync_en, false);
        this->get_parameter_or<double>("common.time_offset_lidar_to_imu", time_diff_lidar_to_imu, 0.0);
        this->get_parameter_or<int>("common.imu_queue_depth", imu_queue_depth_, 2000);
        this->get_parameter_or<int>("common.lidar_queue_depth", lidar_queue_depth_, 50);
        this->get_parameter_or<int>("common.executor_threads", executor_threads_, 3);
        imu_queue_depth_ = std::max(100, imu_queue_depth_);
        lidar_queue_depth_ = std::max(5, lidar_queue_depth_);
        executor_threads_ = std::max(2, executor_threads_);
        this->get_parameter_or<double>("filter_size_corner",filter_size_corner_min,0.5);
        this->get_parameter_or<double>("filter_size_surf",filter_size_surf_min,0.5);
        this->get_parameter_or<double>("filter_size_map",filter_size_map_min,0.5);
        this->get_parameter_or<double>("cube_side_length",cube_len,200.f);
        this->get_parameter_or<float>("mapping.det_range",DET_RANGE,300.f);
        this->get_parameter_or<double>("mapping.fov_degree",fov_deg,180.f);
        this->get_parameter_or<double>("mapping.gyr_cov",gyr_cov,0.1);
        this->get_parameter_or<double>("mapping.acc_cov",acc_cov,0.1);
        this->get_parameter_or<double>("mapping.b_gyr_cov",b_gyr_cov,0.0001);
        this->get_parameter_or<double>("mapping.b_acc_cov",b_acc_cov,0.0001);
        this->get_parameter_or<double>("preprocess.blind", p_pre->blind, 0.01);
        this->get_parameter_or<int>("preprocess.lidar_type", p_pre->lidar_type, AVIA);
        this->get_parameter_or<int>("preprocess.scan_line", p_pre->N_SCANS, 16);
        this->get_parameter_or<int>("preprocess.timestamp_unit", p_pre->time_unit, US);
        this->get_parameter_or<int>("preprocess.scan_rate", p_pre->SCAN_RATE, 10);
        this->get_parameter_or<int>("point_filter_num", p_pre->point_filter_num, 2);
        this->get_parameter_or<bool>("feature_extract_enable", p_pre->feature_enabled, false);
        this->get_parameter_or<bool>("runtime_pos_log_enable", runtime_pos_log, 0);
        this->get_parameter_or<bool>("mapping.extrinsic_est_en", extrinsic_est_en, true);
        this->get_parameter_or<double>("mapping.imu_init_duration", imu_init_duration_, 5.0);
        this->get_parameter_or<double>(
            "mapping.initial_map_duration", initial_map_duration_, 2.0);
        initial_map_duration_ = std::max(0.0, initial_map_duration_);
        this->get_parameter_or<int>(
            "mapping.initial_map_min_voxels", initial_map_min_voxels_, 400);
        initial_map_min_voxels_ = std::max(100, initial_map_min_voxels_);
        this->get_parameter_or<double>(
            "mapping.initial_map_max_duration", initial_map_max_duration_, 10.0);
        initial_map_max_duration_ = std::max(
            initial_map_duration_, initial_map_max_duration_);
        this->get_parameter_or<double>(
            "mapping.safety_max_state_speed", safety_max_state_speed_, 3.0);
        this->get_parameter_or<double>(
            "mapping.safety_max_pose_speed", safety_max_pose_speed_, 3.0);
        this->get_parameter_or<double>(
            "mapping.safety_max_angular_speed", safety_max_angular_speed_, 4.0);
        this->get_parameter_or<double>(
            "mapping.safety_max_position_radius", safety_max_position_radius_, 100.0);
        this->get_parameter_or<double>(
            "mapping.safety_max_absolute_z", safety_max_absolute_z_, 5.0);
        this->get_parameter_or<double>(
            "mapping.safety_max_undistorted_range", safety_max_undistorted_range_, 120.0);
        this->get_parameter_or<double>(
            "mapping.safety_max_invalid_point_fraction",
            safety_max_invalid_point_fraction_, 0.05);
        this->get_parameter_or<int>(
            "mapping.safety_corrupt_cloud_confirmation_samples",
            safety_corrupt_cloud_confirmation_samples_, 3);
        safety_max_undistorted_range_ = std::max(10.0, safety_max_undistorted_range_);
        safety_max_invalid_point_fraction_ = std::clamp(
            safety_max_invalid_point_fraction_, 0.001, 0.50);
        safety_corrupt_cloud_confirmation_samples_ = std::max(
            1, safety_corrupt_cloud_confirmation_samples_);
        this->get_parameter_or<bool>("pcd_save.pcd_save_en", pcd_save_en, false);
        this->get_parameter_or<int>("pcd_save.interval", pcd_save_interval, -1);
        this->get_parameter_or<vector<double>>("mapping.extrinsic_T", extrinT, vector<double>());
        this->get_parameter_or<vector<double>>("mapping.extrinsic_R", extrinR, vector<double>());

        RCLCPP_INFO(this->get_logger(), "p_pre->lidar_type %d", p_pre->lidar_type);

        path.header.stamp = this->get_clock()->now();
        path.header.frame_id = world_frame;

        // /*** variables definition ***/
        // int effect_feat_num = 0, frame_num = 0;
        // double deltaT, deltaR, aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_incre = 0, aver_time_solve = 0, aver_time_const_H_time = 0;
        input_max_age_ = get_parameter("mapping.input_max_age").as_double();
        input_max_imu_gap_ = get_parameter("mapping.input_max_imu_gap").as_double();
        input_max_scan_duration_ = get_parameter("mapping.input_max_scan_duration").as_double();
        input_warmup_duration_ = get_parameter("mapping.input_warmup_duration").as_double();
        stationary_enabled_ = get_parameter("mapping.stationary_constraint").as_bool();
        quality_min_matches_ = get_parameter("mapping.quality_min_matches").as_int();
        quality_min_match_ratio_ = get_parameter("mapping.quality_min_match_ratio").as_double();
        quality_max_residual_ = get_parameter("mapping.quality_max_residual").as_double();
        if (!(input_max_age_ > 0.0) || !(input_max_imu_gap_ > 0.0) ||
            !(input_max_scan_duration_ > 0.0) || !(input_warmup_duration_ >= 0.0) ||
            quality_min_matches_ < 1 || !(quality_min_match_ratio_ > 0.0) ||
            quality_min_match_ratio_ > 1.0 || !(quality_max_residual_ > 0.0)) {
            throw std::runtime_error("Invalid FAST-LIO input/scan-quality guard parameters");
        }

        // bool flg_EKF_converged, EKF_stop_flg = 0;

        FOV_DEG = (fov_deg + 10.0) > 179.9 ? 179.9 : (fov_deg + 10.0);
        HALF_FOV_COS = cos((FOV_DEG) * 0.5 * PI_M / 180.0);

        _featsArray.reset(new PointCloudXYZI());

        memset(point_selected_surf, true, sizeof(point_selected_surf));
        memset(res_last, -1000.0f, sizeof(res_last));
        downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
        downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);
        memset(point_selected_surf, true, sizeof(point_selected_surf));
        memset(res_last, -1000.0f, sizeof(res_last));

        Lidar_T_wrt_IMU<<VEC_FROM_ARRAY(extrinT);
        Lidar_R_wrt_IMU<<MAT_FROM_ARRAY(extrinR);
        p_imu->set_extrinsic(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU);
        p_imu->set_gyr_cov(V3D(gyr_cov, gyr_cov, gyr_cov));
        p_imu->set_acc_cov(V3D(acc_cov, acc_cov, acc_cov));
        p_imu->set_gyr_bias_cov(V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
        p_imu->set_acc_bias_cov(V3D(b_acc_cov, b_acc_cov, b_acc_cov));
        p_imu->set_init_duration(imu_init_duration_);

        fill(epsi, epsi+23, 0.001);
        kf.init_dyn_share(get_f, df_dx, df_dw, h_share_model, NUM_MAX_ITERATIONS, epsi);

        /*** debug record ***/
        // FILE *fp;
        string pos_log_dir = root_dir + "/Log/pos_log.txt";
        fp = fopen(pos_log_dir.c_str(),"w");

        // ofstream fout_pre, fout_out, fout_dbg;
        fout_pre.open(DEBUG_FILE_DIR("mat_pre.txt"),ios::out);
        fout_out.open(DEBUG_FILE_DIR("mat_out.txt"),ios::out);
        fout_dbg.open(DEBUG_FILE_DIR("dbg.txt"),ios::out);
        if (fout_pre && fout_out)
            cout << "~~~~"<<ROOT_DIR<<" file opened" << endl;
        else
            cout << "~~~~"<<ROOT_DIR<<" doesn't exist" << endl;

        /*** ROS subscribe initialization ***/
        imu_callback_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        lidar_callback_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        mapping_callback_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

        rclcpp::SubscriptionOptions imu_options;
        imu_options.callback_group = imu_callback_group_;
        rclcpp::SubscriptionOptions lidar_options;
        lidar_options.callback_group = lidar_callback_group_;

        auto imu_qos = rclcpp::QoS(rclcpp::KeepLast(imu_queue_depth_));
        imu_qos.reliable().durability_volatile();
        auto lidar_qos = rclcpp::QoS(rclcpp::KeepLast(lidar_queue_depth_));
        lidar_qos.best_effort().durability_volatile();
        if (p_pre->lidar_type == AVIA)
        {
            sub_pcl_livox_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
                lid_topic, lidar_qos, livox_pcl_cbk, lidar_options);
        }
        else
        {
            auto pointcloud_qos = rclcpp::SensorDataQoS().keep_last(lidar_queue_depth_);
            sub_pcl_pc_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                lid_topic, pointcloud_qos, standard_pcl_cbk, lidar_options);
        }
        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic, imu_qos, imu_cbk, imu_options);
        // Serialize these callbacks with mapping: the stationary detector and
        // its anchor never race the propagation/matching callback.
        rclcpp::SubscriptionOptions stationary_options;
        stationary_options.callback_group = mapping_callback_group_;
        command_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_guarded", rclcpp::SensorDataQoS().keep_last(1),
            [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
                stationary_guard_.command(steady_seconds(), msg->linear.x,
                    msg->linear.y, msg->angular.z);
            }, stationary_options);
        sport_sub_ = create_subscription<unitree_go::msg::SportModeState>(
            "/lf/sportmodestate", rclcpp::SensorDataQoS().keep_last(1),
            [this](unitree_go::msg::SportModeState::ConstSharedPtr msg) {
                std::array<double, 3> position, rpy;
                for (int i = 0; i < 3; ++i) {
                    position[i] = msg->position[i];
                    rpy[i] = msg->imu_state.rpy[i];
                }
                stationary_guard_.feedback(steady_seconds(), position, rpy,
                    std::hypot(msg->velocity[0], msg->velocity[1]), msg->yaw_speed);
            }, stationary_options);
        pubLaserCloudFull_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered", 20);
        pubLaserCloudFull_body_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered_body", 20);
        pubLaserCloudEffect_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_effected", 20);
        pubLaserCloudMap_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/Laser_map", 20);
        pubOdomAftMapped_ = this->create_publisher<nav_msgs::msg::Odometry>("/Odometry", 20);
        pubPath_ = this->create_publisher<nav_msgs::msg::Path>("/path", 20);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        //------------------------------------------------------------------------------------------------------
        auto period_ms = std::chrono::milliseconds(static_cast<int64_t>(1000.0 / 100.0));
        timer_ = rclcpp::create_timer(
            this, this->get_clock(), period_ms,
            std::bind(&LaserMappingNode::timer_callback, this), mapping_callback_group_);

        auto map_period_ms = std::chrono::milliseconds(static_cast<int64_t>(1000.0));
        map_pub_timer_ = rclcpp::create_timer(
            this, this->get_clock(), map_period_ms,
            std::bind(&LaserMappingNode::map_publish_callback, this), mapping_callback_group_);

        map_save_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "map_save",
            std::bind(
                &LaserMappingNode::map_save_callback, this,
                std::placeholders::_1, std::placeholders::_2),
            rmw_qos_profile_services_default, mapping_callback_group_);

        RCLCPP_INFO(
            this->get_logger(),
            "Node init finished (IMU queue=%d, LiDAR queue=%d, executor threads=%d, IMU init=%.1f s).",
            imu_queue_depth_, lidar_queue_depth_, executor_threads_, imu_init_duration_);
    }

    int executor_threads() const
    {
        return executor_threads_;
    }

    ~LaserMappingNode()
    {
        fout_out.close();
        fout_pre.close();
        if (fp != nullptr) {fclose(fp);}
    }

private:
    void timer_callback()
    {
        if(sync_packages(Measures))
        {
            if (!input_is_healthy()) {
                return;
            }
            if (flg_first_scan)
            {
                first_lidar_time = Measures.lidar_beg_time;
                p_imu->first_lidar_time = first_lidar_time;
                flg_first_scan = false;
                return;
            }

            double t0,t1,t2,t3,t4,t5,match_start, solve_start, svd_time;

            match_time = 0;
            kdtree_search_time = 0.0;
            solve_time = 0;
            solve_const_H_time = 0;
            svd_time   = 0;
            t0 = omp_get_wtime();

            p_imu->Process(Measures, kf, feats_undistort);
            state_point = kf.get_x();
            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;

            if (!feats_undistort || feats_undistort->empty())
            {
                if (p_imu->initializing()) {
                    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                        "IMU initialization in progress (%.1f s stationary window); no deskewed cloud expected yet",
                        imu_init_duration_);
                } else {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                        "Empty deskewed scan: preprocessed=%zu; check HAP returns and robot posture",
                        Measures.lidar->size());
                }
                return;
            }

            // IMU propagation happens before the scan-to-map correction.  Stop
            // here if it has already become implausible so a corrupt deskewed
            // cloud never reaches PCL's integer voxel index calculation.  This
            // is deliberately a non-committing check; the corrected state is
            // committed by the second check immediately before publishing.
            if (ikdtree.Root_Node != nullptr &&
                !state_is_safe(Measures.lidar_end_time, false))
            {
                rclcpp::shutdown();
                return;
            }
            if (!sanitize_undistorted_cloud())
            {
                // An isolated truncated HAP packet is discarded without
                // touching the estimator or map.  sanitize_undistorted_cloud
                // latches safety_faulted_ only after repeated corrupt frames.
                if (safety_faulted_)
                    rclcpp::shutdown();
                return;
            }

            flg_EKF_inited = (Measures.lidar_beg_time - first_lidar_time) < INIT_TIME ? \
                            false : true;
            /*** downsample the feature points in a scan ***/
            downSizeFilterSurf.setInputCloud(feats_undistort);
            downSizeFilterSurf.filter(*feats_down_body);
            t1 = omp_get_wtime();
            feats_down_size = feats_down_body->points.size();
            /*** initialize the map kdtree ***/
            if(ikdtree.Root_Node == nullptr)
            {
                if (!initial_map_collecting_)
                {
                    initial_map_collecting_ = true;
                    initial_map_start_time_ = Measures.lidar_beg_time;
                    initial_map_reference_state_ = state_point;
                    initial_map_cloud_->clear();
                    RCLCPP_INFO(
                        this->get_logger(),
                        "Collecting a %.1f s stationary initial map...",
                        initial_map_duration_);
                }

                for (const auto &point_body : feats_down_body->points)
                {
                    PointType point_world;
                    pointBodyToWorld_ikfom(
                        &point_body, &point_world, initial_map_reference_state_);
                    initial_map_cloud_->push_back(point_world);
                }

                const double collected_duration =
                    Measures.lidar_beg_time - initial_map_start_time_;
                if (collected_duration < initial_map_duration_ ||
                    initial_map_cloud_->size() < 1000)
                {
                    return;
                }

                PointCloudXYZI::Ptr initial_map_downsampled(new PointCloudXYZI());
                downSizeFilterMap.setInputCloud(initial_map_cloud_);
                downSizeFilterMap.filter(*initial_map_downsampled);

                if (initial_map_downsampled->size() <
                    static_cast<std::size_t>(initial_map_min_voxels_) &&
                    collected_duration < initial_map_max_duration_)
                {
                    RCLCPP_WARN_THROTTLE(
                        this->get_logger(), *this->get_clock(), 2000,
                        "Initial map still sparse: %zu/%d voxels after %.2f s; continuing collection",
                        initial_map_downsampled->size(), initial_map_min_voxels_,
                        collected_duration);
                    return;
                }
                if (initial_map_downsampled->size() <
                    static_cast<std::size_t>(initial_map_min_voxels_))
                {
                    RCLCPP_WARN(
                        this->get_logger(),
                        "Initial map reached %.1f s limit with only %zu/%d voxels; "
                        "localization may be weak in this scene",
                        initial_map_max_duration_, initial_map_downsampled->size(),
                        initial_map_min_voxels_);
                }

                state_ikfom reset_state = kf.get_x();
                reset_state.rot = initial_map_reference_state_.rot;
                reset_state.pos = initial_map_reference_state_.pos;
                reset_state.vel = Zero3d;
                kf.change_x(reset_state);
                state_point = kf.get_x();
                pos_lid = state_point.pos +
                    state_point.rot * state_point.offset_T_L_I;

                ikdtree.set_downsample_param(filter_size_map_min);
                ikdtree.Build(initial_map_downsampled->points);
                RCLCPP_INFO(
                    this->get_logger(),
                    "Initial map ready: %.2f s, %zu accumulated -> %zu voxel points "
                    "(%d valid in ikd-tree).",
                    collected_duration, initial_map_cloud_->size(),
                    initial_map_downsampled->size(), ikdtree.validnum());
                initial_map_cloud_->clear();
                return;
            }

            // Do not move/delete the local-map cube while the initial map is
            // still being collected. With a small cube this used to shift the
            // cube on every bootstrap scan and could delete the newly built
            // HAP map before its first scan-to-map update.
            lasermap_fov_segment();
            int featsFromMapNum = ikdtree.validnum();
            kdtree_size_st = ikdtree.size();
            
            // cout<<"[ mapping ]: In num: "<<feats_undistort->points.size()<<" downsamp "<<feats_down_size<<" Map num: "<<featsFromMapNum<<"effect num:"<<effct_feat_num<<endl;

            /*** ICP and iterated Kalman filter update ***/
            if (feats_down_size < 5)
            {
                RCLCPP_WARN(this->get_logger(), "No point, skip this scan!\n");
                return;
            }
            
            normvec->resize(feats_down_size);
            feats_down_world->resize(feats_down_size);

            V3D ext_euler = SO3ToEuler(state_point.offset_R_L_I);
            fout_pre<<setw(20)<<Measures.lidar_beg_time - first_lidar_time<<" "<<euler_cur.transpose()<<" "<< state_point.pos.transpose()<<" "<<ext_euler.transpose() << " "<<state_point.offset_T_L_I.transpose()<< " " << state_point.vel.transpose() \
            <<" "<<state_point.bg.transpose()<<" "<<state_point.ba.transpose()<<" "<<state_point.grav<< endl;

            if(0) // If you need to see map point, change to "if(1)"
            {
                PointVector ().swap(ikdtree.PCL_Storage);
                ikdtree.flatten(ikdtree.Root_Node, ikdtree.PCL_Storage, NOT_RECORD);
                featsFromMap->clear();
                featsFromMap->points = ikdtree.PCL_Storage;
            }

            pointSearchInd_surf.resize(feats_down_size);
            Nearest_Points.resize(feats_down_size);
            int  rematch_num = 0;
            bool nearest_search_en = true; //

            t2 = omp_get_wtime();
            
            /*** iterated state estimation ***/
            double t_update_start = omp_get_wtime();
            double solve_H_time = 0;
            auto predicted_state = kf.get_x();
            auto predicted_covariance = kf.get_P();
            effct_feat_num = 0;
            res_mean_last = std::numeric_limits<double>::infinity();
            kf.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time);
            if (!fast_lio_guard::reliable_match(
                effct_feat_num, feats_down_size, res_mean_last,
                quality_min_matches_, quality_min_match_ratio_, quality_max_residual_))
            {
                // Do not let a weak update teach the map its own drift.
                kf.change_x(predicted_state);
                kf.change_P(predicted_covariance);
                ++consecutive_weak_matches_;
                RCLCPP_ERROR(
                    get_logger(),
                    "Rejected weak scan match (%d/3): matches=%d/%d residual=%.4f m; "
                    "preprocessed=%zu deskewed=%zu minimum_matches=%d; "
                    "odometry/cloud publication and map insertion skipped. "
                    "If Go2 is lying down, stand it up before restarting localization.",
                    consecutive_weak_matches_, effct_feat_num, feats_down_size, res_mean_last,
                    Measures.lidar->size(), feats_undistort->size(), quality_min_matches_);
                if (consecutive_weak_matches_ >= 3) {
                    RCLCPP_FATAL(get_logger(), "FAST-LIO stopped: persistent weak scan matching");
                    rclcpp::shutdown();
                }
                return;
            }
            consecutive_weak_matches_ = 0;
            state_point = kf.get_x();
            // Keep the existing divergence trip effective on the uncorrected
            // state. A stationary observation must not rescue an unsafe state.
            if (!state_is_safe(Measures.lidar_end_time, false)) {
                rclcpp::shutdown();
                return;
            }
            apply_stationary_constraint();
            if (safety_faulted_) {return;}
            state_point = kf.get_x();
            RCLCPP_INFO_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "LIO quality: matches=%d/%d residual=%.4f m state_speed=%.3f m/s",
                effct_feat_num, feats_down_size, res_mean_last, state_point.vel.norm());
            euler_cur = SO3ToEuler(state_point.rot);
            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;
            geoQuat.x = state_point.rot.coeffs()[0];
            geoQuat.y = state_point.rot.coeffs()[1];
            geoQuat.z = state_point.rot.coeffs()[2];
            geoQuat.w = state_point.rot.coeffs()[3];

            if (!state_is_safe(Measures.lidar_end_time))
            {
                // Do not publish the bad pose and, critically, do not insert
                // its transformed points into the ikd-tree. The external
                // localization watchdog will close the Go2 motion gate when
                // odometry stops.
                rclcpp::shutdown();
                return;
            }

            double t_update_end = omp_get_wtime();

            /******* Publish odometry *******/
            publish_odometry(pubOdomAftMapped_, tf_broadcaster_);

            /*** add the feature points to map kdtree ***/
            t3 = omp_get_wtime();
            map_incremental();
            t5 = omp_get_wtime();
            
            /******* Publish points *******/
            if (path_en)                         publish_path(pubPath_);
            if (scan_pub_en)      publish_frame_world(pubLaserCloudFull_);
            if (scan_pub_en && scan_body_pub_en) publish_frame_body(pubLaserCloudFull_body_);
            if (effect_pub_en) publish_effect_world(pubLaserCloudEffect_);
            // if (map_pub_en) publish_map(pubLaserCloudMap_);

            /*** Debug variables ***/
            if (runtime_pos_log)
            {
                frame_num ++;
                kdtree_size_end = ikdtree.size();
                aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t5 - t0) / frame_num;
                aver_time_icp = aver_time_icp * (frame_num - 1)/frame_num + (t_update_end - t_update_start) / frame_num;
                aver_time_match = aver_time_match * (frame_num - 1)/frame_num + (match_time)/frame_num;
                aver_time_incre = aver_time_incre * (frame_num - 1)/frame_num + (kdtree_incremental_time)/frame_num;
                aver_time_solve = aver_time_solve * (frame_num - 1)/frame_num + (solve_time + solve_H_time)/frame_num;
                aver_time_const_H_time = aver_time_const_H_time * (frame_num - 1)/frame_num + solve_time / frame_num;
                T1[time_log_counter] = Measures.lidar_beg_time;
                s_plot[time_log_counter] = t5 - t0;
                s_plot2[time_log_counter] = feats_undistort->points.size();
                s_plot3[time_log_counter] = kdtree_incremental_time;
                s_plot4[time_log_counter] = kdtree_search_time;
                s_plot5[time_log_counter] = kdtree_delete_counter;
                s_plot6[time_log_counter] = kdtree_delete_time;
                s_plot7[time_log_counter] = kdtree_size_st;
                s_plot8[time_log_counter] = kdtree_size_end;
                s_plot9[time_log_counter] = aver_time_consu;
                s_plot10[time_log_counter] = add_point_size;
                time_log_counter ++;
                printf("[ mapping ]: time: IMU + Map + Input Downsample: %0.6f ave match: %0.6f ave solve: %0.6f  ave ICP: %0.6f  map incre: %0.6f ave total: %0.6f icp: %0.6f construct H: %0.6f \n",t1-t0,aver_time_match,aver_time_solve,t3-t1,t5-t3,aver_time_consu,aver_time_icp, aver_time_const_H_time);
                ext_euler = SO3ToEuler(state_point.offset_R_L_I);
                fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " " << euler_cur.transpose() << " " << state_point.pos.transpose()<< " " << ext_euler.transpose() << " "<<state_point.offset_T_L_I.transpose()<<" "<< state_point.vel.transpose() \
                <<" "<<state_point.bg.transpose()<<" "<<state_point.ba.transpose()<<" "<<state_point.grav<<" "<<feats_undistort->points.size()<<endl;
                dump_lio_state_to_log(fp);
            }
        }
    }

    void map_publish_callback()
    {
        if (map_pub_en) publish_map(pubLaserCloudMap_);
    }

    void map_save_callback(std_srvs::srv::Trigger::Request::ConstSharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res)
    {
        RCLCPP_INFO(this->get_logger(), "Saving map to %s...", map_file_path.c_str());
        if (pcd_save_en)
        {
            save_to_pcd();
            res->success = true;
            res->message = "Map saved.";
        }
        else
        {
            res->success = false;
            res->message = "Map save disabled.";
        }
    }

private:
    static double steady_seconds()
    {
        return std::chrono::duration<double>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
    }

    void apply_stationary_constraint()
    {
        if (!stationary_enabled_) {return;}
        bool imu_quiet = !Measures.imu.empty();
        for (const auto &imu : Measures.imu) {
            const auto &g = imu->angular_velocity;
            const auto &a = imu->linear_acceleration;
            const double gyro = std::sqrt(g.x*g.x + g.y*g.y + g.z*g.z);
            const double acceleration = std::sqrt(a.x*a.x + a.y*a.y + a.z*a.z);
            // HAP's raw acceleration uses g. Check every measurement, so an
            // impact or rotation immediately cancels the quiet interval.
            imu_quiet = imu_quiet && std::isfinite(gyro) && gyro < .06 &&
                std::isfinite(acceleration) && acceleration > .85 && acceleration < 1.15;
        }
        if (!stationary_guard_.stationary(steady_seconds(), imu_quiet)) {
            if (stationary_anchored_) {
                RCLCPP_INFO(get_logger(), "Stationary constraint released: motion or stale evidence");
            }
            stationary_anchored_ = false;
            return;
        }
        if (!stationary_anchored_) {
            // Do not anchor an estimator that has already drifted away.
            if (state_point.vel.norm() > .25) {return;}
            stationary_anchor_ = state_point.pos;
            stationary_anchored_ = true;
            RCLCPP_INFO(get_logger(), "Stationary constraint engaged after independent idle evidence");
        }
        if ((state_point.pos - stationary_anchor_).norm() > .20 ||
            state_point.vel.norm() > .50) {
            safety_faulted_ = true;
            RCLCPP_FATAL(get_logger(),
                "Stationary localization diverged; correction refused, restart required");
            rclcpp::shutdown();
            return;
        }
        fast_lio_guard::constrain_stationary(kf, stationary_anchor_);
    }

    bool input_is_healthy()
    {
        std::vector<fast_lio_guard::ImuSample> samples;
        samples.reserve(Measures.imu.size());
        for (const auto & imu : Measures.imu) {
            const auto & a = imu->linear_acceleration;
            const auto & w = imu->angular_velocity;
            samples.push_back({get_time_sec(imu->header.stamp),
                {a.x, a.y, a.z, w.x, w.y, w.z}});
        }
        const char * reason = fast_lio_guard::input_error(
            Measures.lidar_beg_time, Measures.lidar_end_time, now().seconds(), samples,
            last_processed_imu_stamp_, input_max_age_, input_max_imu_gap_,
            input_max_scan_duration_);
        if (reason != nullptr) {
            if (ikdtree.Root_Node != nullptr) {
                RCLCPP_FATAL(get_logger(),
                    "FAST-LIO input safety trip: %s; scan=[%.6f, %.6f] "
                    "imu_count=%zu imu=[%.6f, %.6f] previous_imu=%.6f age=%.3f s; restart required",
                    reason, Measures.lidar_beg_time, Measures.lidar_end_time, samples.size(),
                    samples.empty() ? 0.0 : samples.front().stamp,
                    samples.empty() ? 0.0 : samples.back().stamp,
                    last_processed_imu_stamp_, now().seconds()-Measures.lidar_end_time);
                rclcpp::shutdown();
            } else {
                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 2000,
                    "Waiting for continuous fresh HAP data: %s; initialization reset", reason);
                p_imu->restart_initialization();
                state_ikfom initial_state;
                kf.change_x(initial_state);
                initial_map_collecting_ = false;
                initial_map_cloud_->clear();
                flg_first_scan = true;
                input_ready_since_ = -1.0;
                last_processed_imu_stamp_ = 0.0;
            }
            return false;
        }
        last_processed_imu_stamp_ = samples.back().stamp;
        if (input_ready_since_ < 0.0) {input_ready_since_ = Measures.lidar_beg_time;}
        return Measures.lidar_beg_time - input_ready_since_ >= input_warmup_duration_;
    }

    bool sanitize_undistorted_cloud()
    {
        const std::size_t input_size = feats_undistort->size();
        const double maximum_squared_range =
            safety_max_undistorted_range_ * safety_max_undistorted_range_;
        auto &points = feats_undistort->points;
        const auto new_end = std::remove_if(
            points.begin(), points.end(),
            [maximum_squared_range](const PointType &point) {
                if (!pcl::isFinite(point)) {
                    return true;
                }
                const double squared_range =
                    static_cast<double>(point.x) * point.x +
                    static_cast<double>(point.y) * point.y +
                    static_cast<double>(point.z) * point.z;
                return !std::isfinite(squared_range) || squared_range > maximum_squared_range;
            });
        points.erase(new_end, points.end());
        feats_undistort->width = static_cast<std::uint32_t>(points.size());
        feats_undistort->height = 1;

        const std::size_t rejected = input_size - points.size();
        const double rejected_fraction = input_size == 0 ? 1.0 :
            static_cast<double>(rejected) / static_cast<double>(input_size);
        // One or two invalid returns in a roughly 18k-point HAP frame are
        // routine and were flooding the terminal every two seconds.  Retain a
        // warning for a meaningful cluster; the safety trip below still
        // handles a corrupt frame independently of logging severity.
        if (rejected >= 100 || rejected_fraction >= 0.001) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "Removed %zu/%zu non-finite or over-range deskewed points",
                rejected, input_size);
        } else if (rejected > 0) {
            RCLCPP_DEBUG_THROTTLE(
                this->get_logger(), *this->get_clock(), 5000,
                "Removed %zu/%zu isolated invalid deskewed points",
                rejected, input_size);
        }
        const bool corrupt_cloud = points.size() < 5 ||
            (rejected >= 100 && rejected_fraction > safety_max_invalid_point_fraction_);
        if (corrupt_cloud)
        {
            ++consecutive_corrupt_cloud_samples_;
            if (consecutive_corrupt_cloud_samples_ <
                safety_corrupt_cloud_confirmation_samples_)
            {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "Rejected corrupt deskewed cloud (%d/%d): %zu/%zu points rejected, %.1f%%; "
                    "voxel filtering and map insertion were skipped",
                    consecutive_corrupt_cloud_samples_,
                    safety_corrupt_cloud_confirmation_samples_, rejected,
                    input_size, rejected_fraction * 100.0);
                return false;
            }
            safety_faulted_ = true;
            RCLCPP_FATAL(
                this->get_logger(),
                "FAST-LIO safety trip after %d consecutive corrupt deskewed clouds: "
                "%zu/%zu points rejected, %.1f%%. Restart after checking HAP data.",
                consecutive_corrupt_cloud_samples_, rejected, input_size,
                rejected_fraction * 100.0);
            return false;
        }
        consecutive_corrupt_cloud_samples_ = 0;
        return true;
    }

    bool state_is_safe(double stamp, bool commit = true)
    {
        if (safety_faulted_)
            return false;

        const Eigen::Vector3d position = state_point.pos;
        const Eigen::Vector3d velocity = state_point.vel;
        const Eigen::Matrix3d rotation = state_point.rot.toRotationMatrix();
        const bool finite = position.allFinite() && velocity.allFinite() && rotation.allFinite();
        const double xy_radius = std::hypot(position.x(), position.y());

        const char *reason = nullptr;
        double pose_speed = 0.0;
        double angular_speed = 0.0;
        if (!finite)
        {
            reason = "non-finite state";
        }
        else if (velocity.norm() > safety_max_state_speed_)
        {
            reason = "state velocity exceeded limit";
        }
        else if (xy_radius > safety_max_position_radius_ ||
                 std::abs(position.z()) > safety_max_absolute_z_)
        {
            reason = "position exceeded operating envelope";
        }
        else if (have_last_safe_state_)
        {
            const double dt = stamp - last_safe_stamp_;
            if (dt > 1.0e-4 && dt <= 2.0)
            {
                pose_speed = (position - last_safe_position_).norm() / dt;
                const Eigen::AngleAxisd delta_rotation(last_safe_rotation_.transpose() * rotation);
                angular_speed = std::abs(delta_rotation.angle()) / dt;
                if (pose_speed > safety_max_pose_speed_)
                    reason = "pose jump exceeded limit";
                else if (angular_speed > safety_max_angular_speed_)
                    reason = "rotation jump exceeded limit";
            }
        }

        if (reason != nullptr)
        {
            safety_faulted_ = true;
            RCLCPP_FATAL(
                this->get_logger(),
                "FAST-LIO safety trip: %s (position=[%.3f %.3f %.3f] "
                "state_speed=%.3f pose_speed=%.3f angular_speed=%.3f). "
                "No divergent odometry/cloud will be published; restart after checking HAP data.",
                reason, position.x(), position.y(), position.z(), velocity.norm(),
                pose_speed, angular_speed);
            return false;
        }

        if (commit)
        {
            have_last_safe_state_ = true;
            last_safe_stamp_ = stamp;
            last_safe_position_ = position;
            last_safe_rotation_ = rotation;
        }
        return true;
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull_body_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudEffect_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudMap_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl_pc_;
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_pcl_livox_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_sub_;
    rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr sport_sub_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr map_pub_timer_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr map_save_srv_;
    rclcpp::CallbackGroup::SharedPtr imu_callback_group_;
    rclcpp::CallbackGroup::SharedPtr lidar_callback_group_;
    rclcpp::CallbackGroup::SharedPtr mapping_callback_group_;

    bool effect_pub_en = false, map_pub_en = false;
    int imu_queue_depth_ = 2000;
    int lidar_queue_depth_ = 50;
    int executor_threads_ = 3;
    double imu_init_duration_ = 5.0;
    double initial_map_duration_ = 2.0;
    int initial_map_min_voxels_ = 400;
    double initial_map_max_duration_ = 10.0;
    double input_max_age_ = 0.30;
    double input_max_imu_gap_ = 0.05;
    double input_max_scan_duration_ = 0.15;
    double input_warmup_duration_ = 1.0;
    double input_ready_since_ = -1.0;
    fast_lio_guard::StationaryGuard stationary_guard_;
    bool stationary_enabled_ = false, stationary_anchored_ = false;
    Eigen::Vector3d stationary_anchor_{Eigen::Vector3d::Zero()};
    double last_processed_imu_stamp_ = 0.0;
    int quality_min_matches_ = 50;
    double quality_min_match_ratio_ = 0.05;
    double quality_max_residual_ = 0.15;
    int consecutive_weak_matches_ = 0;
    double safety_max_state_speed_ = 3.0;
    double safety_max_pose_speed_ = 3.0;
    double safety_max_angular_speed_ = 4.0;
    double safety_max_position_radius_ = 100.0;
    double safety_max_absolute_z_ = 5.0;
    double safety_max_undistorted_range_ = 120.0;
    double safety_max_invalid_point_fraction_ = 0.05;
    int safety_corrupt_cloud_confirmation_samples_ = 3;
    int consecutive_corrupt_cloud_samples_ = 0;
    bool safety_faulted_ = false;
    bool have_last_safe_state_ = false;
    double last_safe_stamp_ = 0.0;
    Eigen::Vector3d last_safe_position_{Eigen::Vector3d::Zero()};
    Eigen::Matrix3d last_safe_rotation_{Eigen::Matrix3d::Identity()};
    bool initial_map_collecting_ = false;
    double initial_map_start_time_ = 0.0;
    state_ikfom initial_map_reference_state_;
    PointCloudXYZI::Ptr initial_map_cloud_{new PointCloudXYZI()};
    int effect_feat_num = 0, frame_num = 0;
    double deltaT, deltaR, aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_incre = 0, aver_time_solve = 0, aver_time_const_H_time = 0;
    bool flg_EKF_converged, EKF_stop_flg = 0;
    double epsi[23] = {0.001};

    FILE *fp = nullptr;
    ofstream fout_pre, fout_out, fout_dbg;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    signal(SIGINT, SigHandle);

    auto node = std::make_shared<LaserMappingNode>();
    rclcpp::executors::MultiThreadedExecutor executor(
        rclcpp::ExecutorOptions(), static_cast<size_t>(node->executor_threads()));
    executor.add_node(node);
    executor.spin();

    if (rclcpp::ok())
        rclcpp::shutdown();
    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. pcd save will largely influence the real-time performences **/
    if (pcl_wait_save->size() > 0 && pcd_save_en)
    {
        string file_name = string("scans.pcd");
        string all_points_dir(string(string(ROOT_DIR) + "PCD/") + file_name);
        pcl::PCDWriter pcd_writer;
        cout << "current scan saved to /PCD/" << file_name<<endl;
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
    }

    if (runtime_pos_log)
    {
        vector<double> t, s_vec, s_vec2, s_vec3, s_vec4, s_vec5, s_vec6, s_vec7;    
        FILE *fp2;
        string log_dir = root_dir + "/Log/fast_lio_time_log.csv";
        fp2 = fopen(log_dir.c_str(),"w");
        fprintf(fp2,"time_stamp, total time, scan point size, incremental time, search time, delete size, delete time, tree size st, tree size end, add point size, preprocess time\n");
        for (int i = 0;i<time_log_counter; i++){
            fprintf(fp2,"%0.8f,%0.8f,%d,%0.8f,%0.8f,%d,%0.8f,%d,%d,%d,%0.8f\n",T1[i],s_plot[i],int(s_plot2[i]),s_plot3[i],s_plot4[i],int(s_plot5[i]),s_plot6[i],int(s_plot7[i]),int(s_plot8[i]), int(s_plot10[i]), s_plot11[i]);
            t.push_back(T1[i]);
            s_vec.push_back(s_plot9[i]);
            s_vec2.push_back(s_plot3[i] + s_plot6[i]);
            s_vec3.push_back(s_plot4[i]);
            s_vec5.push_back(s_plot[i]);
        }
        fclose(fp2);
    }

    return 0;
}
