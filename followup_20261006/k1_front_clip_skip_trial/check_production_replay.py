from pathlib import Path
import json,numpy as np
H=Path(__file__).resolve().parent
out=[]
for tag in ['032453','050627','165712','184813','184905','185025','185147']:
 expected=json.loads((H/f'result_{tag}.json').read_text())
 actual=json.loads((H/f'production_result_{tag}.json').read_text())
 for name,rs in actual.items():
  baseline=expected[name];assert len(rs)==len(baseline)
  maxdiff=0.
  for i,(a,b) in enumerate(zip(rs,baseline)):
   assert a['valid']==b['valid'],(tag,name,i,'validity')
   assert a['k1_skip_mask']==b['k1_skip_mask'],(tag,name,i,'skip mask')
   assert a['selected']==b['selected'],(tag,name,i,'selection')
   if a['valid']:
    assert len(a['planned'])==len(b['planned'])
    err=float(np.max(np.abs(np.array(a['planned'])-b['planned'])))
    maxdiff=max(maxdiff,err)
    assert err<1e-6,(tag,name,i,err)
  out.append(dict(tag=tag,mode=name,cycles=len(rs),max_coordinate_difference_m=maxdiff))
(H/'production_validation.json').write_text(json.dumps(out,indent=2))
print('All',sum(r['cycles'] for r in out),'production replay solves match the tested baseline/conditional variant; max difference',max(r['max_coordinate_difference_m'] for r in out))
