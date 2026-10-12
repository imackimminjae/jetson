import json,pickle,numpy as np
from pathlib import Path
from datetime import datetime
root=Path(__file__).resolve().parent.parent;p=Path(__file__).resolve().parent;d=pickle.load(open(root/'drive_debug_20261002_025312_light/analysis/decoded.pkl','rb'));snaps=[];jobs=[];clock=lambda t:datetime.fromtimestamp(t).strftime('%H:%M:%S.%f')[:-3]
for x in d['/debug/upper_branch_event']:
 e=x['data']
 if e['horizon']<1:continue
 tr=min(d['/debug/upper_miqp_trace'],key=lambda x:abs(x['data'][0]-e['t_emit']))['data'];i=len(snaps)
 snaps.append({'event':e,'trace':tr.tolist(),'clock':clock(e['t_start'])})
 for w in [1.,2.]:jobs.append({'id':f'upper_{i}_{w}','mode':'recorded','s':i,'dpsi_weight':w})
(p/'upper_inputs.json').write_text(json.dumps({'snapshots':snaps,'jobs':jobs,'route':d['/navigation/global_path'][0]['xy'].tolist()}))
refs=d['/planner/lower_reference_path/with_arclength'];rows=[]
for x in d['/debug/lower_mpc_trace']:
 a=x['data']
 if a[10]!=1:continue
 candidates=[(i,z) for i,z in enumerate(refs) if z['t']<=a[0]]
 if not candidates:continue
 i,z=candidates[-1];rows.append({'t':a[0],'clock':clock(a[0]),'trace':a.tolist(),'ref':i})
(p/'lower_inputs.json').write_text(json.dumps({'rows':rows,'refs':[{'xy':r['xy'].tolist(),'s':r['s'].tolist()} for r in refs]}))
print('upper snapshots',len(snaps),'lower snapshots',len(rows))
