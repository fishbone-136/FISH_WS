"""Verify latched fault maps on an isolated localhost-only DDS domain."""
import os,sys,time,json
from pathlib import Path
if os.environ.get('ROS_DOMAIN_ID')!='86' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':raise RuntimeError('Isolated domain required')
os.environ['CYCLONEDDS_URI'] = ('<CycloneDDS><Domain><General><AllowMulticast>false</AllowMulticast>'
    '</General><Discovery><ParticipantIndex>auto</ParticipantIndex><Peers>'
    '<Peer Address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>')
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile,DurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool
from obstacle_memory import RememberedObstacles
from environment_terrain_display import EnvironmentTerrainDisplay
rclpy.init();memory=RememberedObstacles();display=EnvironmentTerrainDisplay();fixture=rclpy.create_node('invalid_map_fixture');ex=SingleThreadedExecutor()
for n in [memory,display,fixture]:ex.add_node(n)
q=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
gridpub=fixture.create_publisher(OccupancyGrid,'/traversability/local',q);validpub=fixture.create_publisher(Bool,'/lio/localization_valid',q)
grid=OccupancyGrid();grid.header.frame_id='odom_lio';grid.info.width=grid.info.height=2;grid.info.resolution=.08;grid.info.origin.orientation.w=1.;grid.data=[100,0,90,-1]
memory.memory.mark_xyz([[.2,0.,.1]],(0.,0.,0.),1.);original=memory.memory.cells.copy();received=[];surfaces=[]
def spin(seconds):
 end=time.monotonic()+seconds
 while time.monotonic()<end:ex.spin_once(timeout_sec=.01)
try:
 gridpub.publish(grid);validpub.publish(Bool(data=False));spin(1.3)
 # Subscribe late: stale occupied/free data must have been replaced in DDS.
 fixture.create_subscription(OccupancyGrid,'/safety/navigation_local',lambda m:received.append(m),q)
 fixture.create_subscription(PointCloud2,'/visualization/environment_terrain',lambda m:surfaces.append(m),q)
 spin(1.2)
 assert received and set(received[-1].data)=={-1},'late subscriber retained occupied/free cells'
 assert surfaces and surfaces[-1].width==0,'historical colored overlay retained'
 assert memory.memory.cells==original,'fault must not erase physical memory'
 assert received[-1].info.width==4
 print(json.dumps({'passed':True,'late_subscriber_all_unknown':True,'historical_overlay_empty':True,'physical_memory_preserved':True}))
finally:
 memory.worker.shutdown(wait=True,cancel_futures=True)
 for n in [memory,display,fixture]:n.destroy_node()
 rclpy.shutdown()
