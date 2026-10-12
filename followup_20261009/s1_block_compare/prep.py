# Extract recorded inputs for open-loop replay and closed-loop grid reprojection. Read-only on the bag.
import os,json,gzip,pickle,bisect,numpy as np,rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import OccupancyGrid
os.sched_setaffinity(0,{min(os.sched_getaffinity(0))})
S='/tmp/claude-1000/-home-imac-ros2-ws/0001aebe-4346-4d43-92c3-665e5620a1a1/scratchpad/s1osc/'
D=pickle.load(open(S+'data.pkl','rb'));od,att,es,cs=D['od'],D['att'],D['es'],D['cs']
B='/home/imac/ros2_ws/drive_batches/20261008_182344_scenario1_0cefb5/bag'
r=rosbag2_py.SequentialReader();r.open(rosbag2_py.StorageOptions(uri=B,storage_id='mcap'),rosbag2_py.ConverterOptions('',''));r.set_filter(rosbag2_py.StorageFilter(topics=['/bev/occupancy_grid']))
# open-loop cycles: every upper event within running window
cyc=[]
for a in att:
    for e in es:
        if a['start']<=e['t_start']<a['end'] and e['preview_world']:
            j=int(np.argmin(abs(cs[:,0]-e['t_start'])))
            oi=bisect.bisect_right(od[:,0],e['t_start'])-1
            cyc.append(dict(attempt=a['i'],t=e['t_start'],x=e['x'],y=e['y'],yaw=e['yaw_rad'],v=float(np.hypot(od[oi,4],od[oi,5])),wp0=e['wp0'],wp1=e['wp1'],preview=e['preview_world'],valid=e['valid_plan'],B_before=float(cs[j,1]),B=float(cs[j,2]),published=e['published_world']))
cyc.sort(key=lambda c:c['t']);ci=0;last=None
ol=open('ol_grids.bin','wb');cl=open('cl_grids.bin','wb');clp=[];k=0;n_ol=0
while r.has_next():
    _,b,tn=r.read_next();t=tn/1e9
    while ci<len(cyc) and t>cyc[ci]['t']:
        ol.write(last[1]);cyc[ci]['grid']=n_ol;cyc[ci]['grid_age']=cyc[ci]['t']-last[0];n_ol+=1;ci+=1
    k+=1
    data=bytes(deserialize_message(b,OccupancyGrid).data);last=(t,data)
    if k%2: continue
    a=next((a for a in att if a['start']<=t<a['end']),None)
    if a is None: continue
    j=min(bisect.bisect_left(od[:,0],t),len(od)-1)
    if abs(od[j,0]-t)>0.06: continue
    x,y,yaw=od[j,1:4]
    if not (95<=x<=180 and -75<=y<=20): continue
    cl.write(data);clp.append([x,y,yaw,a['i']])
ol.close();cl.close()
json.dump(cyc,open('ol_cycles.json','w'));json.dump(clp,open('cl_poses.json','w'))
M=pickle.load(open(S+'maps.pkl','rb'));occ=sum(m[0] for m in M['maps'].values());cnt=sum(m[1] for m in M['maps'].values())
p=np.where(cnt>0,occ/np.maximum(cnt,1),1.0);static=np.where(p<0.5,0,100).astype(np.int8)
static.tofile('static_map.bin');json.dump(dict(origin=[M['ext'][0],M['ext'][2]],res=M['res'],width=static.shape[1],height=static.shape[0]),open('static_map.json','w'))
json.dump(dict(route=D['route']),open('route.json','w'))
print('ol cycles',len(cyc),'cl frames',len(clp),'static',static.shape)
