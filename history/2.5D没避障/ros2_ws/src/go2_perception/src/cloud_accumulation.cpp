#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <deque>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace go2_perception
{

class CloudAccumulator final : public rclcpp::Node
{
public:
  CloudAccumulator()
  : Node("cloud_accumulation")
  {
    const auto requested_frames =
      declare_parameter<std::int64_t>("accumulation_frames", 1);
    accumulation_frames_ = static_cast<int>(
      std::clamp<std::int64_t>(requested_frames, 1, 10));
    // Defaults are the validated HAP slice in the sensor frame. Launch files
    // may still override these values for another mounting geometry.
    min_height_ = declare_parameter<double>("min_height", -0.33);
    max_height_ = declare_parameter<double>("max_height", 0.37);
    min_range_ = std::max(0.0, declare_parameter<double>("min_range", 0.25));
    max_range_ = declare_parameter<double>("max_range", 5.5);
    fixed_frame_ = declare_parameter<std::string>("fixed_frame", "livox_frame");
    max_history_age_ = declare_parameter<double>("max_history_age", 0.12);
    const bool use_best_effort = declare_parameter<bool>("use_best_effort", true);

    if (min_height_ >= max_height_) {
      throw std::invalid_argument("min_height must be smaller than max_height");
    }
    if (max_range_ <= min_range_) {
      throw std::invalid_argument("max_range must be greater than min_range");
    }
    if (max_history_age_ < 0.0) {
      throw std::invalid_argument("max_history_age must be non-negative");
    }

    // Small latest-only queues keep dynamic obstacles fresh instead of spending
    // CPU on stale point clouds after a short scheduling spike.
    rclcpp::QoS sensor_qos = rclcpp::SensorDataQoS().keep_last(2);
    if (!use_best_effort) {
      sensor_qos.reliable();
    }

    auto output_qos = sensor_qos;
    output_qos.keep_last(1);
    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("cloud", output_qos);
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "cloud_in", sensor_qos,
      std::bind(&CloudAccumulator::cloud_callback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Cloud accumulator: frames=%d age<=%.2fs height=[%.2f, %.2f] "
      "range=[%.2f, %.2f]m frame=%s",
      accumulation_frames_, max_history_age_, min_height_, max_height_,
      min_range_, max_range_, fixed_frame_.c_str());
  }

private:
  static bool xyz_offsets_for(
    const sensor_msgs::msg::PointCloud2 & cloud,
    std::size_t & x_offset, std::size_t & y_offset, std::size_t & z_offset)
  {
    bool have_x = false;
    bool have_y = false;
    bool have_z = false;
    for (const auto & field : cloud.fields) {
      if (field.datatype != sensor_msgs::msg::PointField::FLOAT32 ||
        field.offset + sizeof(float) > cloud.point_step)
      {
        continue;
      }
      if (field.name == "x") {
        x_offset = field.offset;
        have_x = true;
      } else if (field.name == "y") {
        y_offset = field.offset;
        have_y = true;
      } else if (field.name == "z") {
        z_offset = field.offset;
        have_z = true;
      }
    }
    return have_x && have_y && have_z;
  }

  static std::size_t point_count(const sensor_msgs::msg::PointCloud2 & cloud)
  {
    return static_cast<std::size_t>(cloud.width) * static_cast<std::size_t>(cloud.height);
  }

  static std::int64_t stamp_nanoseconds(const sensor_msgs::msg::PointCloud2 & cloud)
  {
    return static_cast<std::int64_t>(cloud.header.stamp.sec) * 1000000000LL +
           static_cast<std::int64_t>(cloud.header.stamp.nanosec);
  }

  static bool cloud_layout_valid(const sensor_msgs::msg::PointCloud2 & cloud)
  {
    if (cloud.point_step == 0 || cloud.height == 0 || cloud.width == 0) {
      return false;
    }
    const std::size_t packed_row =
      static_cast<std::size_t>(cloud.width) * cloud.point_step;
    const std::size_t row_step = cloud.row_step > 0 ? cloud.row_step : packed_row;
    if (row_step < packed_row) {
      return false;
    }
    const std::size_t required =
      (static_cast<std::size_t>(cloud.height) - 1U) * row_step + packed_row;
    return required <= cloud.data.size();
  }

  void cloud_callback(sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud)
  {
    std::size_t x_offset = 0;
    std::size_t y_offset = 0;
    std::size_t z_offset = 0;
    if (!cloud_layout_valid(*cloud) ||
      !xyz_offsets_for(*cloud, x_offset, y_offset, z_offset))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Ignoring malformed cloud or cloud without float32 x/y/z fields");
      return;
    }

    // Historical frames may only be accumulated in a fixed frame; otherwise
    // robot motion would smear obstacles. HAP therefore defaults to one frame
    // in livox_frame, while a caller may still supply a truly fixed cloud.
    const bool frame_is_fixed = fixed_frame_.empty() || cloud->header.frame_id == fixed_frame_;
    if (!frame_is_fixed) {
      history_.clear();
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "cloud_in frame is '%s', expected fixed frame '%s'; using one frame to avoid motion smear",
        cloud->header.frame_id.c_str(), fixed_frame_.c_str());
    }

    if (!history_.empty() && history_.back()->header.frame_id != cloud->header.frame_id) {
      history_.clear();
    }

    const std::int64_t current_stamp = stamp_nanoseconds(*cloud);
    if (!history_.empty()) {
      const std::int64_t previous_stamp = stamp_nanoseconds(*history_.back());
      if (current_stamp > 0 && previous_stamp > 0 && current_stamp <= previous_stamp) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Ignoring non-monotonic cloud timestamp to protect the TF/SLAM queues");
        return;
      }
      if (current_stamp == 0 || previous_stamp == 0) {
        // A zero-stamped history cannot be transformed at the acquisition time.
        history_.clear();
      }
    }
    history_.push_back(std::move(cloud));
    const auto history_limit = frame_is_fixed ? accumulation_frames_ : 1;
    while (static_cast<int>(history_.size()) > history_limit) {
      history_.pop_front();
    }
    if (current_stamp > 0 && max_history_age_ > 0.0) {
      const auto max_age_ns = static_cast<std::int64_t>(max_history_age_ * 1e9);
      while (history_.size() > 1 &&
        current_stamp - stamp_nanoseconds(*history_.front()) > max_age_ns)
      {
        history_.pop_front();
      }
    }

    publish_filtered_cloud();
  }

  void publish_filtered_cloud()
  {
    const auto & newest = *history_.back();
    auto output = std::make_unique<sensor_msgs::msg::PointCloud2>();
    output->header = newest.header;
    output->height = 1;
    output->fields = newest.fields;
    output->is_bigendian = newest.is_bigendian;
    output->point_step = newest.point_step;
    output->is_dense = false;

    std::size_t candidate_points = 0;
    for (const auto & cloud : history_) {
      if (cloud->point_step == newest.point_step && cloud->fields == newest.fields) {
        candidate_points += point_count(*cloud);
      }
    }
    output->data.reserve(candidate_points * newest.point_step);

    for (const auto & cloud : history_) {
      if (cloud->point_step != newest.point_step || cloud->fields != newest.fields) {
        continue;
      }
      std::size_t x_offset = 0;
      std::size_t y_offset = 0;
      std::size_t z_offset = 0;
      if (!cloud_layout_valid(*cloud) ||
        !xyz_offsets_for(*cloud, x_offset, y_offset, z_offset))
      {
        continue;
      }

      const std::size_t packed_row =
        static_cast<std::size_t>(cloud->width) * cloud->point_step;
      const std::size_t source_row_step = cloud->row_step > 0 ? cloud->row_step : packed_row;
      for (std::size_t row = 0; row < cloud->height; ++row) {
        for (std::size_t column = 0; column < cloud->width; ++column) {
          const auto source_offset = row * source_row_step + column * cloud->point_step;
          float x = 0.0F;
          float y = 0.0F;
          float z = 0.0F;
          std::memcpy(&x, cloud->data.data() + source_offset + x_offset, sizeof(float));
          std::memcpy(&y, cloud->data.data() + source_offset + y_offset, sizeof(float));
          std::memcpy(&z, cloud->data.data() + source_offset + z_offset, sizeof(float));
          const double squared_range =
            static_cast<double>(x) * x + static_cast<double>(y) * y;
          if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
            z < min_height_ || z > max_height_ ||
            squared_range < min_range_ * min_range_ ||
            squared_range > max_range_ * max_range_)
          {
            continue;
          }

          const auto destination_offset = output->data.size();
          output->data.resize(destination_offset + output->point_step);
          std::memcpy(
            output->data.data() + destination_offset,
            cloud->data.data() + source_offset,
            output->point_step);
        }
      }
    }

    output->width = static_cast<std::uint32_t>(output->data.size() / output->point_step);
    output->row_step = output->width * output->point_step;
    publisher_->publish(std::move(output));
  }

  int accumulation_frames_;
  double min_height_;
  double max_height_;
  double min_range_;
  double max_range_;
  double max_history_age_;
  std::string fixed_frame_;
  std::deque<sensor_msgs::msg::PointCloud2::ConstSharedPtr> history_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

}  // namespace go2_perception

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<go2_perception::CloudAccumulator>());
  rclcpp::shutdown();
  return 0;
}
