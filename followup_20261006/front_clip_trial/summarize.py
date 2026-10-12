from pathlib import Path
import json,numpy as np,hashlib
H=Path(__file__).resolve().parent;summary={};total=0
for tag in ['032453','050627','165712','184813','184905','185025','185147']:
 d=json.loads((H/f'result_{tag}.json').read_text());p=H if tag in ['032453','050627','165712'] else H.parent/'origin295_drive_review';cs=json.loads((p/f'input_{tag}.json').read_text())['cycles'];total+=len(cs)*len(d);first=next((i for i,c in enumerate(cs) if not c['rec_valid']),None);row={'cycles':len(cs),'valid_counts':{k:sum(v['valid'] for v in rs) for k,rs in d.items()},'first_recorded_invalid_index':first,'first_invalid_results':{k:rs[first]['valid'] for k,rs in d.items()} if first is not None else {},'adjusted_cycle_counts':{k:sum(bool(v['front_clip_adjustments']) for v in rs) for k,rs in d.items()}};row['same_validity_baseline']=sum(c['rec_valid']==r['valid'] for c,r in zip(cs,d['fixed_m0']));summary[tag]=row
integrity={k:hashlib.sha256(Path(k).read_bytes()).hexdigest()==v for k,v in json.loads((H/'sources_before.json').read_text()).items()};out={'total_solves':total,'results':summary,'production_unchanged':integrity};(H/'summary.json').write_text(json.dumps(out,ensure_ascii=False,indent=2));print('solves',total,'production unchanged',all(integrity.values()))
for tag,row in summary.items():print(tag,row['cycles'],row['valid_counts'],row['first_invalid_results'])
