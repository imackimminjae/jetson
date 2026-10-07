"""Receipt-time map/pose consistency; lag sweep is diagnostic, never a calibration."""
from pathlib import Path
import pickle,json
import numpy as np

here=Path(__file__).resolve().parent;root=here.parents[1]
def rot(a):return np.array([[np.cos(a),-np.sin(a)],[np.sin(a),np.cos(a)]])
out={}
for tag,folder in [('S1','drive_debug_20261006_014143_light'),('S2','drive_debug_20261006_014303_light'),('SC1','drive_debug_20261006_013717_light')]:
    d=pickle.load((root/folder/'analysis/decoded.pkl').open('rb'))
    od=d['/motive/vehicle/odom_map'];ot=np.array([x['stamp'] for x in od]);xy=np.array([x['xy'] for x in od]);yaw=np.unwrap([x['q'][2] for x in od])
    grids=d['/bev/occupancy_grid'];gt=np.array([g['t'] for g in grids])
    def pose(t):return np.array([np.interp(t,ot,xy[:,0]),np.interp(t,ot,xy[:,1])]),np.interp(t,ot,yaw)
    rows=[]
    for lag in [0.,.05,.1,.15,.2,.25,.3,.4,.5]:
        acc={group:[0,0,0,0,0] for group in ['near_turn','near_straight','far_turn']}
        for t in np.arange(max(gt[0],ot[0])+.6,min(gt[-1],ot[-1])-.6,.25):
            i=np.searchsorted(gt,t)-1;j=np.searchsorted(gt,t+.5)-1
            g,h=grids[i],grids[j];p,a=pose(g['t']-lag);q,b=pose(h['t']-lag)
            # Hold the turning/straight cohort fixed across lag hypotheses.
            _,nominal_a=pose(g['t']);_,nominal_b=pose(h['t'])
            turn=abs(nominal_b-nominal_a)>np.radians(4)
            for near in [True,False]:
                if not near and not turn:continue
                gy,gx=np.mgrid[1:128:2,1:128:2];ij=np.c_[gx.ravel(),gy.ravel()]
                body=np.array(g['origin'])+(ij+.5)*g['res']
                m=(body[:,0]>=2)&(body[:,0]<=12 if near else body[:,0]<=30)
                if not near:m&=body[:,0]>12
                ij=ij[m];body=body[m];world=body@rot(a).T+p
                other=(world-q)@rot(b);idx=np.floor((other-np.array(h['origin']))/h['res']).astype(int)
                keep=(idx[:,0]>=0)&(idx[:,0]<128)&(idx[:,1]>=0)&(idx[:,1]<128)
                v=g['grid'][ij[keep,1],ij[keep,0]];z=h['grid'][idx[keep,1],idx[keep,0]]
                known=(v>=0)&(z>=0);f=(v==0)&known;f2=(z==0)&known
                group=('near_turn' if turn else 'near_straight') if near else 'far_turn'
                counts=acc[group];counts[0]+=int(sum(f&f2));counts[1]+=int(sum(f|f2));counts[2]+=int(sum(f&(z>0)));counts[3]+=int(sum(f));counts[4]+=1
        rows.append(dict(lag=lag,groups={k:dict(iou=a[0]/a[1] if a[1] else None,
             free_to_blocked_fraction=a[2]/a[3] if a[3] else None,pairs=a[4]) for k,a in acc.items()}))
    out[tag]=dict(all_stamps_zero=all(g['stamp']==0 for g in grids),rows=rows)
    for group in ['near_turn','near_straight','far_turn']:
        valid=[r for r in rows if r['groups'][group]['iou'] is not None]
        best=max(valid,key=lambda r:r['groups'][group]['iou'])
        print(tag,group,'lag0',rows[0]['groups'][group],'best diagnostic lag',best['lag'],best['groups'][group],flush=True)
(here/'map_consistency.json').write_text(json.dumps(out,indent=2)+'\n')
