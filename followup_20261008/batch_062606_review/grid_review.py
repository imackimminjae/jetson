from pathlib import Path
from collections import Counter
import json,struct,math,os
import numpy as np,rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import OccupancyGrid
H=Path(__file__).resolve().parent;s=json.loads((H/'summary.json').read_text());es=json.loads((H/'events.json').read_text());selected=[]
for i,xx in [(4,238.3),(5,238.5),(5,234.5),(1,239.5)]:
 z=s['attempts'][i-1];e=min([e for e in es if z['start']<=e['t_start']<z['end']],key=lambda e:abs(e['x']-xx));selected.append((i,e))
requests=sorted([(e['t_start'],j) for j,(i,e) in enumerate(selected)]);cursor=0;frames={};times=[];geometry=Counter();nbytes=0
r=rosbag2_py.SequentialReader();r.open(rosbag2_py.StorageOptions(uri=s['session']+'/bag',storage_id='mcap'),rosbag2_py.ConverterOptions('',''));r.set_filter(rosbag2_py.StorageFilter(topics=['/bev/occupancy_grid']));last=None
while r.has_next():
 _,blob,tn=r.read_next();t=tn/1e9;times.append(t);nbytes+=len(blob)
 # Only parse the fixed header for the full scan; decode grid data for selected scenes.
 endian='<' if blob[1]&1 else '>';ln=struct.unpack_from(endian+'I',blob,12)[0];off=16+ln;off=4+((off-4+3)//4)*4
 res,w,h=struct.unpack_from(endian+'fII',blob,off+8);origin_off=4+((off+20-4+7)//8)*8;origin=struct.unpack_from(endian+'7d',blob,origin_off)
 geometry[str((res,w,h,origin,blob[16:16+ln-1].decode()))]+=1
 while cursor<len(requests) and t>requests[cursor][0]:
  if last is not None:frames[requests[cursor][1]]=last
  cursor+=1
 last=(t,blob)
times=np.array(times);gaps=[]
for z in s['attempts']:
 ts=times[(times>=z['start'])&(times<z['end'])];gaps.append(dict(attempt=z['attempt'],count=len(ts),max_gap_sec=float(np.max(np.diff(ts))),gaps_over_0_3=int(np.sum(np.diff(ts)>.3))))
os.environ['MPLCONFIGDIR']='/tmp/batch062606_grid'
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap,BoundaryNorm
from matplotlib import font_manager
font_manager.fontManager.addfont('/usr/share/fonts/truetype/nanum/NanumGothic.ttf');plt.rcParams.update({'font.family':'NanumGothic','axes.unicode_minus':False})
fig,axs=plt.subplots(1,3,figsize=(15,6),layout='constrained');sceneinfo=[]
for j,(i,e) in enumerate(selected):
 gt,blob=frames[j];g=deserialize_message(blob,OccupancyGrid);arr=np.array(g.data).reshape(g.info.height,g.info.width);sceneinfo.append(dict(attempt=i,event=e,grid_receive_t=gt,receive_gap_sec=e['t_start']-gt,origin=[g.info.origin.position.x,g.info.origin.position.y],res=g.info.resolution,shape=arr.shape))
 if j>=3:continue
 ax=axs[j];xx,yy=np.meshgrid(g.info.origin.position.x+np.arange(g.info.width+1)*g.info.resolution,g.info.origin.position.y+np.arange(g.info.height+1)*g.info.resolution);yaw=e['yaw_rad'];c=math.cos(yaw);sn=math.sin(yaw);wx=e['x']+c*xx-sn*yy;wy=e['y']+sn*xx+c*yy
 ax.set_facecolor('#c7d4e1');ax.pcolormesh(wx,wy,arr,cmap=ListedColormap(['#c7d4e1','white','#777e84']),norm=BoundaryNorm([-2,-.5,.5,101],3),shading='flat');route=np.array(s['route']);ax.plot(*route.T,'k--',lw=1,label='지정 경로');ref=np.array(e['preview_world']);ax.plot(*ref.T,':o',c='#8c56ad',ms=3,label='이전 계획에서 만든 참조');plan=np.array(e['published_world']);
 if len(plan):ax.plot(*plan.T,'-o',c='#c84b36' if i==5 else '#098774',ms=3,label='이번 발행 경로')
 for row in e['intervals_world']:
  if int(row[0])==6:ax.plot([row[2],row[4]],[row[3],row[5]],lw=4,c='#257dd2',label='k6 허용 단면' if row[1]==0 else None)
 ax.scatter(e['x'],e['y'],marker='^',c='black',s=50);ax.set_xlim(230,279);ax.set_ylim(120,161);ax.set_aspect('equal');ax.set_xlabel('X (m)');ax.set_ylabel('Y (m)');ax.set_title(['4회차: 북쪽 후보가 함께 남음','5회차: 남쪽 후보만 남음','5회차 직전: 참조 포함 도로도 축소'][j]);ax.legend(fontsize=7,loc='lower left')
fig.savefig(H/'branch_comparison.png',dpi=150);plt.close(fig)
out=dict(grid_count=len(times),raw_grid_bytes=nbytes,geometry=geometry,run_gaps=gaps,scenes=sceneinfo)
(H/'grid_review.json').write_text(json.dumps(out,indent=2)+'\n');print('geometry',geometry);print('gaps',gaps)
