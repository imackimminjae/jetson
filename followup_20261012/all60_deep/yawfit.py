import sys,json,numpy as np,rosbag2_py,collections
from rclpy.serialization import deserialize_message
from nav_msgs.msg import OccupancyGrid,Odometry
from std_msgs.msg import String
from scipy import ndimage
RD='/home/imac/ros2_ws/followup_20261009/route_design/';M=np.load(RD+'road_mask.npy');ext=json.load(open(RD+'road_mask.json'))['ext']
DIN=ndimage.distance_transform_edt(~M)*0.25  # distance outside road (0 inside)
g=.3125;GX,GY=np.meshgrid((np.arange(128)+.5)*g,-20+(np.arange(128)+.5)*g)
def score(p,d,dyaw,dx=0,dy=0):
    y=p[2]+dyaw;wx=p[0]+dx+np.cos(y)*GX-np.sin(y)*GY;wy=p[1]+dy+np.sin(y)*GX+np.cos(y)*GY;free=d<=1
    ix=np.clip(((wx-ext[0])/.25).astype(int),0,M.shape[1]-1);iy=np.clip(((wy-ext[2])/.25).astype(int),0,M.shape[0]-1)
    return np.mean(DIN[iy[free],ix[free]])   # mean distance of BEV-free cells from processed road (lower=better)
def run(bag,every):
    r=rosbag2_py.SequentialReader();r.open(rosbag2_py.StorageOptions(uri=bag,storage_id='mcap'),rosbag2_py.ConverterOptions('',''));r.set_filter(rosbag2_py.StorageFilter(topics=['/bev/occupancy_grid','/motive/vehicle/odom_map','/knu_batch/active_route']))
    pose=None;k=0;res=[];route='?'
    while r.has_next():
        n,b,t=r.read_next()
        if n=='/knu_batch/active_route':
            try: route=__import__('json').loads(deserialize_message(b,String).data).get('route',route)
            except Exception: pass
        elif n=='/motive/vehicle/odom_map':
            m=deserialize_message(b,Odometry);pose=(m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.orientation.z,np.hypot(m.twist.twist.linear.x,m.twist.twist.linear.y))
        elif pose is not None and pose[3]>2:
            k+=1
            if k%every: continue
            d=np.frombuffer(bytes(deserialize_message(b,OccupancyGrid).data),dtype=np.int8).reshape(128,128)
            if (d<=1).sum()<500: continue
            best=min(((score(pose,d,np.radians(a)),a) for a in np.arange(-15,15.1,1.0)))
            res.append((t,route,pose[0],pose[1],best[1],best[0],score(pose,d,0)))
    return res
if __name__=="__main__":
  out={}
  
  for name,bag,ev in [('1008_s1','/home/imac/ros2_ws/drive_batches/20261008_182344_scenario1_0cefb5/bag',40),('1008_s3','/home/imac/ros2_ws/drive_batches/20261008_185426_scenario3_72c4df/bag',40),('1009_all','/home/imac/ros2_ws/drive_batches/20261009_070714_all_d8c3cf/bag',40)]:
      R=run(bag,ev);out[name]=R;a=np.array([x[4] for x in R]);print(name,'frames',len(R),'best yaw offset deg: median %.1f p10 %.1f p90 %.1f'%(np.median(a),np.percentile(a,10),np.percentile(a,90)),flush=True)
      if name=='1009_all':
          by=collections.defaultdict(list)
          for x in R: by[x[1]].append(x[4])
          for k in sorted(by,key=lambda s:int(s[8:]) if s.startswith('scenario') else 99): print('   ',k,'median %.1f (n=%d)'%(np.median(by[k]),len(by[k])))
  __import__('pickle').dump(out,open('yawfit.pkl','wb'))
  