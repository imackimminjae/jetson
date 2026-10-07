import pickle,numpy as np,json
from pathlib import Path
from datetime import datetime
h=Path(__file__).resolve().parent;root=h.parents[1];extra={}
for tag in ['171507','174526']:
 d=pickle.load((root/f'drive_debug_20261006_{tag}_light/analysis/decoded.pkl').open('rb'));es=[v['data'] for v in d['/debug/upper_branch_event']];g=[v for v in d['/bev/occupancy_grid'] if es[0]['t_start']<=v['t']<=es[-1]['t_start']];extra[tag]={'upper_active_grid_frames':len(g),'active_near9m_empty':int(sum(not np.any(v['grid'][:,:28]==0) for v in g)),'active_40m_edge_free':int(sum(np.any(v['grid'][:,-1]==0) for v in g)),'last_traces':[]}
 for m in d['/debug/upper_miqp_trace'][-7:]:
  a=m['data'];extra[tag]['last_traces'].append({'time':datetime.fromtimestamp(a[0]).strftime('%H:%M:%S.%f')[:-3],'N':int(a[7]),'valid':bool(a[8]),'previous_seed_used':bool(a[-1]),'turn_window_step':int(a[14])})
(h/'additional_checks.json').write_text(json.dumps(extra,indent=2));print(json.dumps(extra,indent=2))
