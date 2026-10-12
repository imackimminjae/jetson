import sys,json,gzip,pickle,bisect,numpy as np,rosbag2_py,matplotlib;matplotlib.use('Agg');import matplotlib.pyplot as plt
from rclpy.serialization import deserialize_message
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
def frames(bag,target,tol=1.5):
    r=rosbag2_py.SequentialReader();r.open(rosbag2_py.StorageOptions(uri=bag,storage_id='mcap'),rosbag2_py.ConverterOptions('',''));r.set_filter(rosbag2_py.StorageFilter(topics=['/bev/occupancy_grid','/motive/vehicle/odom_map']))
    pose=None;out=[]
    while r.has_next():
        n,b,t=r.read_next()
        if n=='/motive/vehicle/odom_map':
            m=deserialize_message(b,Odometry);pose=(m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.orientation.z)
        elif pose is not None:
            for tg in target:
                if np.hypot(pose[0]-tg[0],pose[1]-tg[1])<tol:
                    g=deserialize_message(b,OccupancyGrid);out.append((tg,pose,np.frombuffer(bytes(g.data),dtype=np.int8).reshape(128,128)));break
    return out
T=[(248,138),(253,137)]
A=frames('/home/imac/ros2_ws/drive_batches/20261008_185426_scenario3_72c4df/bag',T)
B=frames('/home/imac/ros2_ws/drive_batches/20261009_070714_all_d8c3cf/bag',T)
print(len(A),len(B))
fig,axs=plt.subplots(2,4,figsize=(20,10))
for row,(F,lab) in enumerate([(A,'10-08'),(B,'10-09')]):
    for col,tg in enumerate(T):
        cand=[f for f in F if f[0]==tg]
        for j,f in enumerate(cand[:2]):
            ax=axs[row,col*2+j];tgt,p,d=f
            # world-frame render of this single frame
            g=.3125;GX,GY=np.meshgrid((np.arange(128)+.5)*g,-20+(np.arange(128)+.5)*g);wx=p[0]+np.cos(p[2])*GX-np.sin(p[2])*GY;wy=p[1]+np.sin(p[2])*GX+np.cos(p[2])*GY
            ax.scatter(wx[d<=1],wy[d<=1],s=1,c='k');ax.plot(p[0],p[1],'r^',ms=8);ax.set_xlim(240,300);ax.set_ylim(110,180);ax.set_aspect('equal');ax.grid(alpha=.3)
            ax.set_title(f"{lab} pose ({p[0]:.1f},{p[1]:.1f}) yaw {np.degrees(p[2]):.0f}",fontsize=10)
fig.savefig('n43_same_view.png',dpi=60,bbox_inches='tight')
