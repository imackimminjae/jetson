from pathlib import Path
import os,csv,json
os.environ['MPLCONFIGDIR']='/tmp/bias009_plot_mpl'
import numpy as np, yaml, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
R=Path('/home/imac/ros2_ws');O=Path(__file__).resolve().parent
runs=['20261009_091539_scenario1_1f86ff','20261009_091900_scenario2_ddfdb0','20261009_092246_scenario3_4bb1ae']
back=R/'followup_20261009/batch_070714_review/configs';mask=np.load(back/'road_backdrop.npz')['road_mask'];ext=json.loads((back/'road_mask.json').read_text())['ext']
colors=['#0072B2','#009E73','#CC79A7']
figall,axes=plt.subplots(1,3,figsize=(18,6));sources=[]
with PdfPages(O/'bias009_trajectories.pdf') as pdf:
 for ri,run in enumerate(runs):
  b=R/'drive_batches'/run;c=yaml.safe_load((b/'effective_config.yaml').read_text())['upper_planner_node']['ros__parameters'];assert c['enable_waypoint_bias'] and abs(c['waypoint_yaw_bias_rad']-.09)<1e-8
  n=int(run.split('_scenario')[1][0]);rows=list(csv.DictReader(next((b/'results').glob('*/summary.csv')).open()));odom=[];ph=[];prev=None
  rd=rosbag2_py.SequentialReader();rd.open(rosbag2_py.StorageOptions(uri=str(b/'bag'),storage_id='mcap'),rosbag2_py.ConverterOptions('',''));names=['/motive/vehicle/odom_map','/knu_batch/status'];rd.set_filter(rosbag2_py.StorageFilter(topics=names));types={x.name:get_message(x.type) for x in rd.get_all_topics_and_types() if x.name in names}
  while rd.has_next():
   topic,blob,tn=rd.read_next();m=deserialize_message(blob,types[topic]);t=tn/1e9
   if topic==names[0]:odom.append([t,m.pose.pose.position.x,m.pose.pose.position.y])
   else:
    v=json.loads(m.data);key=(v['index'],v['sequence'])
    if key!=prev:ph.append(v);prev=key
  od=np.asarray(odom);path=b/'routes'/f'scenario{n}.csv';path=path if path.exists() else R/f'src/virtual_control/data/knu_routes/scenario{n}.csv';route=np.array([[float(x['x']),float(x['y'])] for x in csv.DictReader(path.open())]);theta=.09;rot=np.array([[np.cos(theta),-np.sin(theta)],[np.sin(theta),np.cos(theta)]]);biased=(route-route[0])@rot.T+route[0]+[c['waypoint_x_bias_m'],c['waypoint_y_bias_m']];tracks=[]
  for i,row in enumerate(rows,1):
   states=[p for p in ph if p['index']==i];start=next((p['phase_stamp_sec'] for p in states if p['phase']=='running'),None)
   if start is None:continue
   end=next((p['phase_stamp_sec'] for p in states if p['phase'] in ['coasting','aborting']),od[-1,0]);stop=next((p['phase_stamp_sec'] for p in ph if p['index']==i+1),od[-1,0]);tr=od[(od[:,0]>=start)&(od[:,0]<=stop)];tracks.append((i,row['outcome'],end,tr));sources.append(dict(run_id=run,scenario=n,attempt=i,start_epoch_s=start,end_epoch_s=end,yaw_bias_rad=.09))
  def draw(ax):
   ax.imshow(mask,origin='lower',extent=ext,cmap='Greys',vmin=0,vmax=1,alpha=.23);ax.plot(route[:,0],route[:,1],'k--o',ms=3,lw=1,label='Nominal waypoints');ax.plot(biased[:,0],biased[:,1],color='#E69F00',ls='--',lw=1.2,label='Biased waypoints (+0.09 rad)');points=[route,biased]
   for i,outcome,end,tr in tracks:
    if len(tr)==0:continue
    k=tr[:,0]<end;color=colors[(i-1)%len(colors)];ax.plot(tr[k,1],tr[k,2],c=color,lw=1.5,label=f'A{i}');ax.plot(tr[~k,1],tr[~k,2],c=color,ls=':',lw=1);points.append(tr[:,1:3])
    if k.any():ax.plot(*tr[k][-1,1:3],'x',c=color,ms=7)
   ax.plot(*route[0],'g^',ms=8,label='Start');ax.plot(*route[-1],'k*',ms=10,label='Nominal goal');p=np.vstack(points);lo=p.min(0)-12;hi=p.max(0)+12;mid=(lo+hi)/2;half=max(hi-lo)/2;ax.set_xlim(mid[0]-half,mid[0]+half);ax.set_ylim(mid[1]-half,mid[1]+half);ax.set_aspect('equal');ax.grid(alpha=.2);ax.set_xlabel('map X [m]');ax.set_ylabel('map Y [m]');ax.set_title(f'Scenario {n} | yaw bias +0.09 rad (5.16 deg)\n{run}',fontsize=9);ax.legend(fontsize=7)
  fig,ax=plt.subplots(figsize=(9,8));draw(ax);fig.text(.05,.015,'Solid: running | dotted: coasting | x: end of running | road: reconstructed design backdrop',fontsize=8);fig.tight_layout(rect=(0,.03,1,1));pdf.savefig(fig);name=f'scenario{n}'+('_earlier_session' if ri==3 else '');fig.savefig(O/f'{name}.png',dpi=200);plt.close(fig)
  if ri<3:draw(axes[ri])
 figall.tight_layout();figall.savefig(O/'scenarios123.png',dpi=200);plt.close(figall)
(O/'sources.json').write_text(json.dumps(sources,indent=2)+'\n');print('Plots ready',flush=True)
