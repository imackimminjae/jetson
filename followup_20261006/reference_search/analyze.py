from pathlib import Path
from datetime import datetime
from zoneinfo import ZoneInfo
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.transforms import Affine2D
from matplotlib.colors import ListedColormap

here=Path(__file__).resolve().parent
summary={'original_matches':0,'counterfactuals':{},'critical_cycles':[]}
limit=np.tan(.30)*.75*6./2.8

def assess(r,c,grid):
    if not r['valid']:return {'valid':False}
    p=np.asarray(r['planned']);seg=np.diff(p,axis=0)
    heading=np.unwrap(np.arctan2(seg[:,1],seg[:,0]));turn=np.abs(np.diff(heading))
    xy=np.array([c['x'],c['y']]);a=c['yaw'];rot=np.array([[np.cos(a),-np.sin(a)],[np.sin(a),np.cos(a)]])
    # Centreline samples at <=0.1 m: an extra rejection check, not a footprint guarantee.
    samples=np.concatenate([u+np.linspace(0,1,max(2,int(np.ceil(np.linalg.norm(v-u)/.1))+1))[:,None]*(v-u)
        for u,v in zip(p[:-1],p[1:])])
    body=(samples-xy)@rot;gx=np.floor(body[:,0]/.3125).astype(int);gy=np.floor((body[:,1]+20)/.3125).astype(int)
    inside=(gx>=0)&(gx<128)&(gy>=0)&(gy<128);values=grid[gy[inside],gx[inside]]
    return dict(valid=True,endpoint=p[-1].tolist(),max_segment_turn_deg=float(np.degrees(turn.max(initial=0))),
        exceeds_existing_turn_check=bool(np.any(turn>limit+1e-3)),
        sampled_blocked=int(np.sum(values>0)),sampled_unknown=int(np.sum(values<0)),sampled_outside=int(np.sum(~inside)),
        sample_count=len(samples))

for tag in ['S1','S2','R3','A']:
    folder=here.parent/('sc3_0141_0143' if tag.startswith('S') else 'verify')
    inp=json.loads((folder/f'replay_{tag}.json').read_text())
    grids=np.fromfile(folder/f'replay_{tag}_grids.bin',dtype=np.int8).reshape(-1,128,128)
    data=json.loads((here/f'probe_{tag}.json').read_text())
    old=json.loads((here.parent/'interval_consistency'/f'replay_{tag}_out.json').read_text())
    for mode,rows in data.items():
        expected=old[mode+'_0.000000'];assert len(rows)==len(expected)
        for i,(row,ref) in enumerate(zip(rows,expected)):
            original=row['probes']['original']
            assert original['valid']==ref['valid']
            assert not ref['valid'] or np.allclose(original['planned'],ref['planned'],atol=1e-10,rtol=0)
            summary['original_matches']+=1
            c=inp['cycles'][i];grid=grids[c['grid_index']]
            checked={k:assess(v,c,grid) for k,v in row['probes'].items()}
            for name,value in checked.items():
                key=f'{tag}/memory{mode}/{name}'
                s=summary['counterfactuals'].setdefault(key,dict(total=0,valid=0,turn_check_reject=0,map_check_reject=0))
                s['total']+=1;s['valid']+=int(value['valid'])
                if value['valid']:
                    s['turn_check_reject']+=int(value['exceeds_existing_turn_check'])
                    s['map_check_reject']+=int(any(value[f'sampled_{k}'] for k in ['blocked','unknown','outside']))
            if tag=='S2' and mode=='1' and i>=17:
                summary['critical_cycles'].append(dict(index=i,time=datetime.fromtimestamp(row['t'],ZoneInfo('Asia/Seoul')).isoformat(),
                    B_before=row['B_before'],probes=checked))
    if tag=='S2':
        fig,axes=plt.subplots(1,2,figsize=(12,6),sharex=True,sharey=True)
        i=20;c=inp['cycles'][i];g=grids[c['grid_index']];rows=data['1'][i]['probes'];route=np.array(inp['route'])
        for ax,(name,title) in zip(axes,[('same_seed_original_inset','Previous-plan seed'),('fresh_straight','Independent waypoint seed')]):
            transform=Affine2D().rotate(c['yaw']).translate(c['x'],c['y'])+ax.transData
            colors=np.where(g<0,1,np.where(g>0,2,0))
            ax.imshow(colors,origin='lower',extent=(0,40,-20,20),transform=transform,cmap=ListedColormap(['#f2f2ed','#9eacb8','#353e48']),vmin=0,vmax=2,interpolation='nearest')
            r=rows[name]
            for candidates in r['cands']:
                for a,b,x,y in candidates:ax.plot([a,x],[b,y],color='#ab80d5',alpha=.6,lw=1)
            ref=np.array(r['reference']);plan=np.array(r['planned'])
            ax.plot(route[:,0],route[:,1],':',color='#0089cb',lw=2,label='Waypoint route')
            ax.plot(ref[:,0],ref[:,1],'--',color='#e68b2c',lw=1.6,label='Search reference')
            ax.plot(plan[:,0],plan[:,1],'-o',color='#069d7a',lw=2.4,ms=3,label='QP output')
            ax.scatter([c['x']],[c['y']],marker='^',s=90,color='#dd424a',label='Recorded pose',zorder=5)
            ax.set_title(title+'\nEndpoint '+str(tuple(np.round(plan[-1],1))))
            ax.set_aspect('equal');ax.set_xlim(237,283);ax.set_ylim(122,163);ax.set_xlabel('Map east (m)')
            ax.grid(alpha=.15)
        axes[0].set_ylabel('Map north (m)');axes[1].legend(loc='lower left',fontsize=8)
        fig.suptitle('SC3 01:43:24 — same recorded BEV, pose, B=0.5 and original interval rules\nSingle-cycle diagnostic; neither output was driven',fontsize=11)
        fig.tight_layout();fig.savefig(here/'seed_comparison.png',dpi=150);plt.close(fig)

assert summary['original_matches']==228
summary['total_qp_pipeline_evaluations']=summary['original_matches']*4
summary['segment_turn_limit_deg']=float(np.degrees(limit))
(here/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
print('Original replay paths matched:',summary['original_matches'])
for key,value in summary['counterfactuals'].items():
    if key.startswith('S2/memory1'):print(key,value)
for row in summary['critical_cycles']:
    if row['index'] in [20,22,23,24,25,26]:print(row)
