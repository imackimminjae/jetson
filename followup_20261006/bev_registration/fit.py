from pathlib import Path
import pickle,json,numpy as np
from scipy.ndimage import distance_transform_edt,map_coordinates,binary_erosion
from scipy.optimize import least_squares
from scipy.spatial import cKDTree
H=Path(__file__).resolve().parent;R=H.parents[1];res=.3125

def load(tag):
 d=pickle.load((R/f'drive_debug_20261006_{tag}_light/analysis/decoded.pkl').open('rb'));o=d['/motive/vehicle/odom_map'];ot=np.array([x['t'] for x in o]);xx=np.array([x['xy'] for x in o]);aa=np.unwrap([x['q'][2] for x in o]);vel=np.array([np.linalg.norm(x['v'][:2]) for x in o]);out=[]
 for g in d['/bev/occupancy_grid'][::8]:
  t=g['t'];j=np.searchsorted(ot,t)-1
  if j<0 or j>=len(ot)-1 or vel[j]<2 or t-ot[j]>.10:continue
  p=np.array([np.interp(t,ot,xx[:,i]) for i in range(2)]);a=np.interp(t,ot,aa);road=g['grid']==0;known=g['grid']>=0
  if road.sum()<300:continue
  edge=(road!=np.roll(road,1,axis=0))|(road!=np.roll(road,1,axis=1));edge&=binary_erosion(known,iterations=2);edge[:3,:]=False;edge[-3:,:]=False;edge[:,:3]=False;edge[:,-3:]=False
  iy,ix=np.where(edge);pts=np.array([(ix+.5)*res,(iy+.5)*res-20]).T
  # Common interior excludes changes of visibility border from fitting objective.
  pts=pts[(pts[:,0]>5)&(pts[:,0]<25)&(abs(pts[:,1])<10)][::3]
  if len(pts)<15:continue
  dist=distance_transform_edt(~edge)*res
  out.append({'t':t,'p':p,'a':a,'pts':pts,'dist':dist,'grid':g['grid'],'tag':tag})
 return out
old=sum([load(t) for t in ['032453','032545','045314']],[])
new={t:load(t) for t in ['180803','181932']};pairs=[]
for tag,fr in new.items():
 used=set()
 for n in fr:
  costs=np.array([np.linalg.norm(o['p']-n['p'])+8*abs(np.arctan2(np.sin(o['a']-n['a']),np.cos(o['a']-n['a']))) for o in old]);oi=int(np.argmin(costs));o=old[oi];dist=np.linalg.norm(o['p']-n['p']);angle=abs(np.degrees(np.arctan2(np.sin(o['a']-n['a']),np.cos(o['a']-n['a']))));bucket=(int(n['p'][0]/2),int(n['p'][1]/2))
  if dist>1.5 or angle>5 or bucket in used:continue
  used.add(bucket);pairs.append((n,o))

def rot(a):return np.array([[np.cos(a),-np.sin(a)],[np.sin(a),np.cos(a)]])
def residual(v,ps):
 ox,oy,sx,sy,dyaw=v;out=[]
 for n,o in ps:
  b=n['pts']*np.array([sx,sy])+[ox,oy];w=b@rot(n['a']+dyaw).T+n['p'];ob=(w-o['p'])@rot(o['a']);uv=np.array([(ob[:,1]+20)/res-.5,ob[:,0]/res-.5]);dd=map_coordinates(o['dist'],uv,order=1,mode='constant',cval=6);known=map_coordinates((o['grid']>=0).astype(float),uv,order=0,mode='constant',cval=0);dd=np.where(known>.5,dd,6);out.extend(dd)
 return np.array(out)
models={'current':([0,0,1,1,0],[]),'origin295':([2.95,0,1,1,0],[]),'x_only':([2.95,0,1,1,0],[0]),'xy':([2.95,0,1,1,0],[0,1]),'xy_scale':([2.95,0,1,1,0],[0,1,2,3]),'xy_scale_yaw':([2.95,0,1,1,0],[0,1,2,3,4])}
result={'pairs': [{'new_tag':n['tag'],'new_t':n['t'],'old_tag':o['tag'],'old_t':o['t'],'new_pose':[*n['p'],n['a']],'old_pose':[*o['p'],o['a']],'points':len(n['pts'])} for n,o in pairs],'models':{}}
print('pairs',len(pairs),'points',sum(len(n['pts']) for n,o in pairs),flush=True)
for name,(init,indices) in models.items():
 base=np.array(init,float)
 if indices:
  bounds=np.array([[-5,10],[-5,5],[.7,1.3],[.7,1.3],[-.15,.15]])[indices].T
  def fun(x):
   v=base.copy();v[indices]=x;return residual(v,pairs)
  opt=least_squares(fun,base[indices],bounds=bounds,loss='soft_l1',f_scale=.4,diff_step=.01,max_nfev=160);base[indices]=opt.x
 rr=residual(base,pairs);record={'parameters':base.tolist(),'median_m':float(np.median(rr)),'p90_m':float(np.quantile(rr,.9)),'mean_m':float(rr.mean()),'by_tag':{}}
 for tag in new:
  r=residual(base,[(n,o) for n,o in pairs if n['tag']==tag]);record['by_tag'][tag]={'median_m':float(np.median(r)),'p90_m':float(np.quantile(r,.9))}
 result['models'][name]=record;print(name,record,flush=True)
(H/'fit.json').write_text(json.dumps(result,indent=2));pickle.dump(pairs,(H/'pairs.pkl').open('wb'))
