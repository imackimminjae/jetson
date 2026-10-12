import numpy as np,rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import OccupancyGrid,Odometry
for name,bag in [('10-08 S1','/home/imac/ros2_ws/drive_batches/20261008_182344_scenario1_0cefb5/bag'),('10-08 S3','/home/imac/ros2_ws/drive_batches/20261008_185426_scenario3_72c4df/bag'),('10-09 all','/home/imac/ros2_ws/drive_batches/20261009_070714_all_d8c3cf/bag')]:
    r=rosbag2_py.SequentialReader();r.open(rosbag2_py.StorageOptions(uri=bag,storage_id='mcap'),rosbag2_py.ConverterOptions('',''));r.set_filter(rosbag2_py.StorageFilter(topics=['/bev/occupancy_grid','/motive/vehicle/odom_map']))
    lat=[];olat=[];k=0;gt=[]
    while r.has_next():
        n,b,t=r.read_next();t=t/1e9
        if n=='/bev/occupancy_grid':
            k+=1
            if k%5: continue
            g=deserialize_message(b,OccupancyGrid);s=g.header.stamp.sec+g.header.stamp.nanosec/1e9;lat.append(t-s);gt.append(t)
        else:
            if len(olat)<20000 and k%3==0:
                m=deserialize_message(b,Odometry);olat.append(t-(m.header.stamp.sec+m.header.stamp.nanosec/1e9))
    lat=np.array(lat);olat=np.array(olat);gi=np.diff(np.array(gt))/5
    print(f"{name}: BEV receive-stamp median {np.median(lat)*1000:.0f} ms p90 {np.percentile(lat,90)*1000:.0f} ms | odom receive-stamp median {np.median(olat)*1000:.0f} ms | BEV rate ~{1/np.median(gi):.1f} Hz")
