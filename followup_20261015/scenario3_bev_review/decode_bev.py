from pathlib import Path
import sys,json
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import OccupancyGrid
P=Path(__file__).resolve().parent
for name in sys.argv[1:]:
 b=Path(name).resolve();r=rosbag2_py.SequentialReader();r.open(rosbag2_py.StorageOptions(uri=str(b/'bag'),storage_id='mcap'),rosbag2_py.ConverterOptions('',''));r.set_filter(rosbag2_py.StorageFilter(topics=['/bev/occupancy_grid']))
 count=0;times=[];grids=[];metadata=[];last=-float('inf');stamps=[]
 while r.has_next():
  n,data,tn=r.read_next();count+=1;t=tn/1e9
  if t-last<.18:continue
  m=deserialize_message(data,OccupancyGrid);q=m.info.origin.orientation
  info=dict(frame=m.header.frame_id,resolution=m.info.resolution,width=m.info.width,height=m.info.height,origin=[m.info.origin.position.x,m.info.origin.position.y],quaternion=[q.x,q.y,q.z,q.w]);metadata.append(info)
  times.append(t);stamps.append(m.header.stamp.sec+m.header.stamp.nanosec/1e9);grids.append(np.asarray(m.data,dtype=np.int8).reshape(m.info.height,m.info.width));last=t
 np.savez_compressed(P/f'bev_{b.name}.npz',t=times,stamp=stamps,grids=np.asarray(grids));(P/f'bev_{b.name}_metadata.json').write_text(json.dumps(dict(session_dir=str(b),total_recorded=count,selected=len(times),selection_min_interval_sec=.18,per_frame=metadata),indent=2)+'\n')
 print(b.name,'raw',count,'selected',len(times),'info',metadata[0],'values',np.unique(np.asarray(grids),return_counts=True),flush=True)
