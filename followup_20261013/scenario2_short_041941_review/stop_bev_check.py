"""Inspect four final stop grids without accumulated maps or live ROS nodes."""
from pathlib import Path
import csv
import json
import math
import os
os.environ['ROS_DOMAIN_ID']='177'
os.environ['ROS_LOCALHOST_ONLY']='1'
os.environ['MPLCONFIGDIR']='/tmp/s2_short_stop_mpl'
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import OccupancyGrid
import matplotlib
matplotlib.use('Agg')
from matplotlib import pyplot as plt,font_manager
from matplotlib.colors import ListedColormap,BoundaryNorm
OUT=Path(__file__).resolve().parent
rows=list(csv.DictReader((OUT/'per_attempt.csv').open()))
data=np.load(OUT/'scenario2/signals.npz')['odom']
font_manager.fontManager.addfont('/usr/share/fonts/truetype/nanum/NanumGothic.ttf')
plt.rcParams.update({'font.family':'NanumGothic','axes.unicode_minus':False})
r=rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri='/home/imac/ros2_ws/drive_batches/20261013_041941_scenario2_173ca0/bag',storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
r.set_filter(rosbag2_py.StorageFilter(topics=['/bev/occupancy_grid']))
records=[]
fig,ax=plt.subplots(figsize=(10,6.5))
for a in rows:
    if a['outcome']!='completed':continue
    target=float(a['stop_phase_epoch_s'])-.2
    r.seek(int((target-1)*1e9));last=None
    while r.has_next():
        _,blob,t=r.read_next()
        if t>int(target*1e9):break
        last=blob,t
    assert last is not None
    blob,t=last;g=deserialize_message(blob,OccupancyGrid)
    raster=np.asarray(g.data,dtype=np.int8).reshape(g.info.height,g.info.width)
    pose=data[np.argmin(abs(data[:,0]-t/1e9))]
    q=g.info.origin.orientation
    gyaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    def at_body(x,y):
        dx,dy=x-g.info.origin.position.x,y-g.info.origin.position.y
        c,s=math.cos(gyaw),math.sin(gyaw)
        ix,iy=math.floor((c*dx+s*dy)/g.info.resolution),math.floor((-s*dx+c*dy)/g.info.resolution)
        return int(raster[iy,ix]) if 0<=ix<g.info.width and 0<=iy<g.info.height else None
    values=[at_body(x,y) for x in [1,2,3,4,5] for y in [-1,0,1]]
    rec={'attempt':int(a['attempt']),'grid_epoch_s':t/1e9,'pose_epoch_s':float(pose[0]),
         'grid_to_pose_receive_difference_ms':1000*float(pose[0]-t/1e9),
         'pose_map':pose[1:5].tolist(),'ahead_body_sample_x_m':[1,2,3,4,5],
         'ahead_body_sample_y_m':[-1,0,1],'values':values,
         'drivable_samples':sum(v is not None and 0<=v<=1 for v in values),'samples':15}
    records.append(rec)
    np.savez_compressed(OUT/f'stop_grid_a{a["attempt"]}.npz',data=raster)
    if a['attempt']!='1':continue
    gx,gy=np.meshgrid(np.arange(g.info.width+1)*g.info.resolution,np.arange(g.info.height+1)*g.info.resolution)
    c,s=math.cos(gyaw),math.sin(gyaw)
    bx=g.info.origin.position.x+c*gx-s*gy;by=g.info.origin.position.y+s*gx+c*gy
    c,s=math.cos(pose[3]),math.sin(pose[3])
    wx,wy=pose[1]+c*bx-s*by,pose[2]+s*bx+c*by
    cats=np.where(raster<0,2,np.where(raster<=1,0,1))
    ax.pcolormesh(wx,wy,cats,cmap=ListedColormap(['white','#575757','#e8cf8b']),
                  norm=BoundaryNorm([-.5,.5,1.5,2.5],3),shading='flat',rasterized=True)
    start,end,stop=map(float,[a['start_epoch_s'],a['outcome_epoch_s'],a['stop_phase_epoch_s']])
    tr=data[(data[:,0]>=start)&(data[:,0]<stop)]
    run=tr[:,0]<end
    ax.plot(tr[run,1],tr[run,2],color='#0072B2',lw=2,label='running')
    ax.plot(tr[~run,1],tr[~run,2],':',color='#0072B2',lw=2,label='목표 접근 후 이동')
    ax.plot(-76,67,'*',color='#009E73',ms=14,label='새 목표 (-76,67)')
    ax.plot(pose[1],pose[2],'k^',ms=9,label='정지 확인 직전 위치')
    ax.set_title('시나리오2 · 목표 접근1회 · 정지 확인 직전 BEV\n목표에서 약35.8m 지난 위치 · 전방15표본 중 주행 가능0개')
ax.set(xlim=(-95,5),ylim=(45,100),xlabel='map X [m]',ylabel='map Y [m]')
ax.set_aspect('equal');ax.set_facecolor('#e6eaf0');ax.grid(alpha=.15);ax.legend(fontsize=9)
fig.text(.04,.03,'배경은 정지 확인 직전 단일 BEV를 수신 시각 인접 odom pose로 표시.\n옅은 청회색은 BEV 밖, 짙은 회색은 비주행 셀. 과거 궤적과 배경은 동시각이 아니며 실제 충돌 판정이 아님.',fontsize=8)
fig.tight_layout(rect=(0,.08,1,1))
fig.savefig(OUT/'post_completion_stop_bev.png',dpi=170)
fig.savefig(OUT/'post_completion_stop_bev.pdf')
plt.close(fig)
(OUT/'stop_bev_checks.json').write_text(json.dumps(records,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(records,ensure_ascii=False))
