import json,sys,numpy as np
sys.path.insert(0,'/home/imac/ros2_ws/followup_20261006/scripts')
from analyze_runs import project
GP=np.array([[187,220],[187,192],[195,168],[215,157],[235,142],[255,135],[270,158],[278,172]],float)
J=np.array([257.0,139.0])
def cls(P):
    v=np.array(P[-1])-J; a=np.degrees(np.arctan2(v[1],v[0])); return ('NE' if 25<a<90 else 'E'),a
for tag in sys.argv[1:]:
    inp=json.load(open(f'replay_{tag}.json'))['cycles']
    print('#####',tag)
    import os
    OXS=os.environ.get('OXS','0,2.95').split(',')
    res={ox:json.load(open(f'result_{tag}_ox{ox}.json')) for ox in OXS}
    # reproduction (fixed mode '1_0.000000', origin 0) vs recorded plan
    rr=res['0']['1_0.000000']; err=[];vm=0
    for c,r in zip(inp,rr):
        vm+= (c['rec_valid']==r['valid'])
        if c['rec_valid'] and r['valid'] and len(c['rec_planned'])==len(r['planned']): err.append(np.abs(np.array(c['rec_planned'])-np.array(r['planned'])).max())
    print(f'   reproduction origin0 fixed: valid match {vm}/{len(inp)}, plan max err median {np.median(err):.3f} m, p90 {np.quantile(err,.9):.3f} m')
    for mode,name in (('0_0.000000','carried'),('1_0.000000','fixed')):
        line=[]
        for ox in OXS:
            seq=[]
            for c,r in zip(inp,res[ox][mode]):
                prog=float(project(GP,np.array([c['x']]),np.array([c['y']]))[0][0])
                if not (100<=prog<=124): continue
                seq.append('x' if not r['valid'] else ('N' if cls(r['planned'])[0]=='NE' else 'e'))
            rec=''.join('x' if not c['rec_valid'] else ('N' if cls(c['rec_planned'])[0]=='NE' else 'e') for c in inp if 100<=float(project(GP,np.array([c['x']]),np.array([c['y']]))[0][0])<=124)
            line.append(f'ox{ox}:{"".join(seq)}')
        print(f'   {name:7s} prog100-124  recorded:{rec}  '+'  '.join(line))
