from pathlib import Path
import pickle,json,numpy as np
p=Path(__file__).resolve().parent;d=pickle.load(open(p.parent/'drive_debug_20261003_055920_light/analysis/decoded.pkl','rb'));od=d['/motive/vehicle/odom_map'];ot=np.array([x['stamp'] for x in od]);op=np.array([x['xy'] for x in od]);oy=np.unwrap([x['q'][2] for x in od]);gr=d['/bev/occupancy_grid'];gt=np.array([x['t'] for x in gr]);base=np.arange(ot[0]+1,ot[-1]-1,.5)
def pose(t):return np.array([np.interp(t,ot,op[:,0]),np.interp(t,ot,op[:,1])]),np.interp(t,ot,oy)
def rot(y):return np.array([[np.cos(y),-np.sin(y)],[np.sin(y),np.cos(y)]])
y,x=np.mgrid[1:128:2,1:128:2];ij=np.stack([x.ravel(),y.ravel()],axis=1);rows=[]
for shift in [0.,1.,2.,2.95,4.,-2.95]:
 for lag in [0.,.05,.1,.15,.2,.3]:
  intersections=unions=0;scores=[]
  for t in base:
   i=np.searchsorted(gt,t)-1;j=np.searchsorted(gt,t+.5)-1;g,h=gr[i],gr[j];pos,yaw=pose(g['t']-lag);pos2,yaw2=pose(h['t']-lag)
   if abs(yaw2-yaw)<np.radians(4):continue
   points=np.array(g['origin'])+(ij+.5)*g['res']+[shift,0];world=points@rot(yaw).T+pos;body=(world-pos2)@rot(yaw2)-[shift,0];index=np.floor((body-np.array(h['origin']))/h['res']).astype(int)
   mask=(index[:,0]>=0)&(index[:,0]<128)&(index[:,1]>=0)&(index[:,1]<128);v=g['grid'][ij[mask,1],ij[mask,0]];z=h['grid'][index[mask,1],index[mask,0]];known=(v>=0)&(z>=0);a=(v==0)&known;b=(z==0)&known;it=np.sum(a&b);un=np.sum(a|b)
   if un>30:intersections+=it;unions+=un;scores.append(it/un)
  rows.append({'origin_x_added_m':shift,'assumed_receipt_lag_s':lag,'turning_pair_count':len(scores),'free_space_iou':intersections/unions if unions else None})
print('top hypotheses',sorted(rows,key=lambda x:x['free_space_iou'] or 0,reverse=True)[:8])
print('fixed0 lag',[x for x in rows if x['assumed_receipt_lag_s']==0])
out={'all_grid_header_stamps_zero':all(x['stamp']==0 for x in gr),'comparison':'Static-world temporal free-space consistency on turning pairs. Receipt-time alignment, not ground-truth calibration. Offset/lag may trade off.','results':rows}
(p/'grid_consistency.json').write_text(json.dumps(out,indent=2))
