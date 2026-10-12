import json,pickle
from pathlib import Path
import numpy as np
from datetime import datetime
p=Path(__file__).resolve().parent
d=pickle.load(open(p.parent/'drive_debug_20261001_184521_light/analysis/decoded.pkl','rb'))
g=d['/bev/occupancy_grid'];gt=np.array([x['t'] for x in g]);snapshots=[]
for record in d['/debug/upper_branch_event']:
 e=record['data'];tr=min(d['/debug/upper_miqp_trace'],key=lambda x:abs(x['data'][0]-e['t_emit']))['data']
 b=min(d['/debug/upper_interval_centering_state'],key=lambda x:abs(x['data'][0]-e['t_emit']))['data']
 assert abs(tr[0]-e['t_emit'])<.05 and abs(b[0]-e['t_emit'])<.05
 previous=[v for v in d['/planner/upper_path_sparse'] if v['t']<e['t_start']]
 snapshots.append({'event':e,'trace':tr.tolist(),'B_before':float(b[1]),
                   'previous_world':previous[-1]['xy'].tolist() if previous else [],
                   'grid':int(np.searchsorted(gt,e['t_start'],side='right')-1),
                   'clock':datetime.fromtimestamp(e['t_start']).strftime('%H:%M:%S.%f')[:-3]})
targets=['18:45:32.474','18:45:34.474','18:45:39.674','18:45:39.874','18:45:40.074','18:45:40.474','18:45:41.474','18:45:41.674','18:45:41.874','18:45:42.074']
selected=sorted({min(range(len(snapshots)),key=lambda i:abs(snapshots[i]['event']['t_start']-datetime.fromisoformat('2026-10-01T'+t).timestamp())) for t in targets})
jobs=[];grid_ids=set()
for i,s in enumerate(snapshots):jobs.append({'id':f'recorded_{i}','mode':'recorded','s':i,'repeat':20 if i in selected else 1})
for i in selected:
 for weight in [0.0,.15,.3]:jobs.append({'id':f'turn_{i}_{weight}','mode':'recorded','s':i,'turn_weight':weight})
 for weight in [2.0,5.0]:jobs.append({'id':f'dpsi_{i}_{weight}','mode':'recorded','s':i,'dpsi_weight':weight})
 t=snapshots[i]['event']['t_start']
 for j in np.flatnonzero((gt>=t-.2)&(gt<=t+.06)):
  grid_ids.add(int(j));jobs.append({'id':f'match_{i}_{j}','mode':'pipeline','s':i,'grid':int(j),'no_solve':True})
export_grids={str(i):{'t':g[i]['t'],'origin':g[i]['origin'],'q':g[i]['q'],'res':g[i]['res'],'width':g[i]['width'],'height':g[i]['height'],'data':g[i]['grid'].reshape(-1).tolist()} for i in grid_ids}
(p/'inputs.json').write_text(json.dumps({'snapshots':snapshots,'grids':export_grids,'route':d['/navigation/global_path'][0]['xy'].tolist(),'selected':selected,'jobs':jobs}))
print('snapshots',len(snapshots),'grids',len(export_grids),'jobs',len(jobs),'selected',[(i,snapshots[i]['clock']) for i in selected])
