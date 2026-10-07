from pathlib import Path
import json
import numpy as np
from scipy.signal import butter,sosfiltfilt
here=Path(__file__).resolve().parent
summary={'upper_replay':{},'upper_screen':[],'lower_response':[],'closed_loop':{},'baseline_matches':{}}
replay_matches=0
for cfg in json.loads((here/'configs.json').read_text()):
    name=cfg['name']
    for tag in ['S1','S2','R3','A']:
        folder=here.parent/('sc3_0141_0143' if tag.startswith('S') else 'verify')
        cycles=json.loads((folder/f'replay_{tag}.json').read_text())['cycles']
        xy=np.array([[c['x'],c['y']] for c in cycles]);progress=np.r_[0,np.cumsum(np.linalg.norm(np.diff(xy,axis=0),axis=1))]
        rows=json.loads((here/f'replay_{name}_{tag}.json').read_text())['0_0.000000']
        near=[];angle=[]
        for i,r in enumerate(rows):
            if not r['valid'] or not r['used_previous'] or (tag.startswith('S') and not 35<=progress[i]<=110):continue
            a=np.array(r['planned']);b=np.array(r['reference'])
            if len(a)<4:continue
            near.append(float(np.max(np.linalg.norm(a[1:3]-b[1:3],axis=1))))
            u=a[1]-a[0];v=b[1]-b[0];angle.append(float(abs(np.degrees(np.arctan2(np.cross(v,u),v@u)))))
        z=dict(failures=sum(not r['valid'] for r in rows),near_p95_m=float(np.quantile(near,.95)),heading_p95_deg=float(np.quantile(angle,.95)))
        summary['upper_replay'][name+'/'+tag]=z
        if tag.startswith('S'):print('UPPER',name,tag,z)
        if name=='baseline':
            old=json.loads((here.parent/'interval_consistency'/f'replay_{tag}_out.json').read_text())['0_0.000000']
            assert len(rows)==len(old)
            for a,b in zip(rows,old):
                assert a['valid']==b['valid'] and (not a['valid'] or np.allclose(a['planned'],b['planned'],atol=1e-10,rtol=0))
                replay_matches+=1
summary['baseline_matches']['replay']=replay_matches
for l in (here/'results_synthetic_screen.jsonl').read_text().splitlines():
    r=json.loads(l);summary['upper_screen'].append(dict(scene=r['scene'],config=r['config']['name'],outcome=r['outcome'],failures=sum(not p['valid'] for p in r['plans'])))
for l in [line for f in sorted(here.glob('results_lower_recorded*.jsonl')) for line in f.read_text().splitlines()]:
    r=json.loads(l);a=r['states'];t=np.array([s['t'] for s in a]);mask=(t>=4)&(t<=t[-1]-.5)
    cmd=np.degrees([s['command'] for s in a]);error=np.array([s['original_ref_error'] for s in a])
    hp=sosfiltfilt(butter(2,.3,btype='highpass',fs=10,output='sos'),cmd)
    z=dict(scene=r['scene'],plant=r['plant']['name'],config=r['config']['name'],outcome=r['outcome'],
        steering_highpass_rms_deg=float(np.sqrt(np.mean(hp[mask]**2))),tracking_p95_m=float(np.quantile(error[mask],.95)),
        tracking_max_m=float(np.max(error[mask])),saturation_percent=float(100*np.mean(abs(cmd[mask])>=np.degrees(.298))))
    summary['lower_response'].append(z)
    print('LOWER',z)
prior=(here.parent/'near_consistency/analyze_sim.py').read_text();ns={'np':np}
exec(prior[prior.index('def branch_result('):prior.index('\nsummary={}')],ns)
classify=ns['branch_result'];matches=0
for group in ['synthetic','replica','ysynthetic','robust']:
    files=sorted(here.glob(f'results_{group}_configs_lower*.jsonl'))
    if not files:continue
    old={}
    for l in (here.parent/'verify'/f'results_{group}_prod.jsonl').read_text().splitlines():
        r=json.loads(l)
        if r['config']['name']=='installed_yaml':old[(r['scene'],r['plant']['name'])]=r
    for l in [line for f in files for line in f.read_text().splitlines()]:
        r=json.loads(l);name=r['config']['name'];k=group+'/'+name
        z=summary['closed_loop'].setdefault(k,dict(total=0,correct=0,failures=0,bad=[]))
        ok=classify(r);z['total']+=1;z['correct']+=int(ok);z['failures']+=sum(not p['valid'] for p in r['plans'])
        if not ok:z['bad'].append(dict(scene=r['scene'],plant=r['plant']['name'],outcome=r['outcome']))
        if name=='baseline':
            b=old[(r['scene'],r['plant']['name'])]
            assert r['outcome']==b['outcome'] and len(r['states'])==len(b['states'])
            assert all(np.allclose([u[k] for k in ['x','y','yaw','command']],[v[k] for k in ['x','y','yaw','command']],atol=1e-10,rtol=0) for u,v in zip(r['states'],b['states']))
            matches+=1
summary['baseline_matches']['closed_loop']=matches
for k,v in summary['closed_loop'].items():print('CLOSED',k,v)
(here/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
assert replay_matches==114
assert matches==56
print('Baseline reproduced:',summary['baseline_matches'])
