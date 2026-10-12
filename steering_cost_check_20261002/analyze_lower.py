import json,numpy as np
from pathlib import Path
p=Path(__file__).resolve().parent;r=[json.loads(s) for s in (p/'lower_results.jsonl').read_text().splitlines()];rows=[]
for x in r:
 v={int(t['weight']):t for t in x['trials']};rows.append({'t':x['t'],'clock':x['clock'],'recorded':x['recorded'],'s240':v[240]['steering'],'s120':v[120]['steering'],'s60':v[60]['steering'],'valid':all(t['valid'] for t in v.values())})
errors=np.array([abs(x['s240']-x['recorded']) for x in rows]);matched=[x for x,e in zip(rows,errors) if e<1e-5];print('baseline reproduced',len(matched),'/',len(rows),'max_deg',np.degrees(max(errors)))
for a,b in [('02:53:13','02:53:35'),('02:53:25','02:53:33.700'),('02:53:32','02:53:35')]:
 z=[x for x in matched if a<=x['clock']<b]
 print('WINDOW',a,b,'n',len(z))
 for w in [60,120]:
  dif=np.degrees([x[f's{w}']-x['s240'] for x in z]);print(w,'abs difference med,p95,max deg',np.quantile(abs(dif),[.5,.95,1]))
for x in matched:
 if '02:53:33.200'<=x['clock']<='02:53:35.000':print(x['clock'],*[round(np.degrees(x[k]),3) for k in ['s240','s120','s60']])
(p/'lower_comparison.json').write_text(json.dumps({'baseline_matched':len(matched),'total':len(rows),'baseline_max_error_deg':float(np.degrees(max(errors))),'rows':rows},indent=2))
