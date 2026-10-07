from pathlib import Path
import json,numpy as np,pickle
H=Path(__file__).resolve().parent;R=H.parents[1];cycles=[];grids=[]
for tag in ['175433','174526']:
 cs=json.loads((H/f'{tag}_input.json').read_text())['cycles'];rs=json.loads((H/f'{tag}_full40_result.json').read_text())['fixed'];gg=pickle.load((R/f'drive_debug_20261006_{tag}_light/analysis/decoded.pkl').open('rb'))['/bev/occupancy_grid']
 for idx,(c,r) in enumerate(zip(cs,rs)):
  if not c['rec_valid'] or not r['valid']:continue
  err=np.max(np.linalg.norm(np.array(c['rec_planned'])-r['planned'],axis=1))
  if err<=.001:continue
  for delta in range(-2,3):
   j=c['grid_record_index']+delta
   if not 0<=j<len(gg):continue
   cc=dict(c,grid_index=len(grids),source_tag=tag,source_cycle=idx,grid_delta=delta,grid_age_sec=c['t']-gg[j]['t']);cycles.append(cc);grids.append(gg[j]['grid'].astype(np.int8))
route=json.loads((H/'175433_input.json').read_text())['route'];(H/'alignment_input.json').write_text(json.dumps({'route':route,'cycles':cycles}));np.stack(grids).tofile(H/'alignment_grids.bin');(H/'alignment_specs.json').write_text(json.dumps([{'name':'fixed','legacy':1}]))
