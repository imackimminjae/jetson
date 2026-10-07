"""Keep the current raw section fixed; isolate contraction and B update effects."""
from pathlib import Path
from datetime import datetime
from zoneinfo import ZoneInfo
import pickle,json
import numpy as np

here=Path(__file__).resolve().parent;root=here.parents[1];out={}
for tag,folder in [('S1','drive_debug_20261006_014143_light'),('S2','drive_debug_20261006_014303_light')]:
    d=pickle.load((root/folder/'analysis/decoded.pkl').open('rb'))
    rows=[]
    for item in d['/debug/upper_branch_event']:
        e=item['data'];t=e['t_start']
        if not (190<=e['x']<=245 and 140<=e['y']<=180) or not e['valid_plan']:continue
        state=min(d['/debug/upper_interval_centering_state'],key=lambda q:abs(q['t']-t))['data']
        assert abs(state[0]-t)<.01
        info=min(d['/debug/upper_interval_info'],key=lambda q:abs(q['t']-t))['data'].reshape(-1,12)
        r=dict(time=datetime.fromtimestamp(t,ZoneInfo('Asia/Seoul')).isoformat(),B_before=state[1],B=state[2],steps=[])
        for k in [1,2]:
            point=np.array(e['preview_world'][k]);candidates=[]
            for c in e['intervals_world']:
                if c[0]!=k:continue
                a,b=np.array(c[2:4]),np.array(c[4:6]);axis=(b-a)/np.linalg.norm(b-a)
                rawL=c[6];inset=(c[6]-c[7])/2;rawa=a-inset*axis
                proj=float((point-rawa)@axis)
                gap=lambda ins:float(max(0.,ins-proj,proj-(rawL-ins)))
                row=info[(info[:,0]==k)&(info[:,1]==c[1])][0]
                previous_on=rawL<= (9. if row[8] else 1.5*state[1])
                previous_inset=.3*rawL if previous_on else min(2.,.45*rawL)
                candidates.append(dict(candidate=c[1],raw_length=rawL,clipped=bool(row[7]),
                    inset=inset,raw_gap=gap(0),processed_gap=gap(inset),
                    previous_B_gap=gap(previous_inset),previous_B_inset=previous_inset,
                    fixed_margin_gap=gap(min(2.,.45*rawL)),
                    continuous_min_gap=gap(min(.3*rawL,2.,.45*rawL))))
            r['steps'].append(dict(k=k,candidates=candidates,
                **{name:min(c[name] for c in candidates) for name in ['raw_gap','processed_gap','previous_B_gap','fixed_margin_gap','continuous_min_gap']}))
        rows.append(r)
    out[tag]=rows
    for k in [1,2]:
        values=[r['steps'][k-1] for r in rows]
        print(tag,'k',k,'cycles',len(values),'forced>.5',sum(v['processed_gap']>.5 for v in values),
              'B_effect>.05',sum(abs(v['processed_gap']-v['previous_B_gap'])>.05 for v in values),
              'raw_gap>.3125',sum(v['raw_gap']>.3125 for v in values))
    for r in rows:
        s=r['steps'][0]
        if s['processed_gap']>.5:
            print(r['time'][11:19],{key:round(s[key],3) for key in ['raw_gap','processed_gap','previous_B_gap','fixed_margin_gap','continuous_min_gap']})
(here/'attribution.json').write_text(json.dumps(out,indent=2)+'\n')
