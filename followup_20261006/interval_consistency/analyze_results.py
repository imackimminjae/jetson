from pathlib import Path
from collections import defaultdict
import importlib.util,json,numpy as np

here=Path(__file__).resolve().parent
# Reuse only the branch classifier, without executing the previous report script.
prior=(here.parent/'near_consistency/analyze_sim.py').read_text()
namespace={'np':np};exec(prior[prior.index('def branch_result('):prior.index('\nsummary={}')],namespace)
classify=namespace['branch_result'];summary={};checks=[]
for group in ['synthetic','replica','ysynthetic','robust']:
    rows=[]
    for suffix in ['', '_transition']:
        rows.extend(json.loads(l) for l in (here/f'results_{group}{suffix}.jsonl').read_text().splitlines())
    original={}
    for l in (here.parent/'verify'/f'results_{group}_prod.jsonl').read_text().splitlines():
        r=json.loads(l)
        if r['config']['name']=='installed_yaml':original[(r['scene'],r['plant']['name'])]=r
    for r in rows:
        key=group+'/'+r['config']['name']
        z=summary.setdefault(key,dict(correct=0,total=0,failures=0,mean_saturation_percent=0.,bad=[]))
        ok=classify(r);z['correct']+=int(ok);z['total']+=1;z['failures']+=sum(not p['valid'] for p in r['plans'])
        z['mean_saturation_percent']+=100*float(np.mean([abs(s['command'])>=.298 for s in r['states']]))
        if not ok:z['bad'].append([r['scene'],r['plant']['name'],r['outcome']])
        if r['config']['interval_mode']==0:
            old=original[(r['scene'],r['plant']['name'])]
            same=r['outcome']==old['outcome'] and len(r['states'])==len(old['states']) and all(
                np.allclose([s[k] for k in ['x','y','yaw','command']], [t[k] for k in ['x','y','yaw','command']],atol=1e-10,rtol=0)
                for s,t in zip(r['states'],old['states']))
            checks.append(same)
for k,z in summary.items():
    z['mean_saturation_percent']/=z['total'];print(k,z)
replay={};replay_checks=[]
for tag in ['S1','S2','R3','A']:
    folder=here.parent/('sc3_0141_0143' if tag.startswith('S') else 'verify')
    cycles=json.loads((folder/f'replay_{tag}.json').read_text())['cycles']
    rows_by_key=json.loads((here/f'replay_{tag}_out.json').read_text())
    old=json.loads((here.parent/'near_consistency'/f'replay_{tag}_out.json').read_text())['0_0.000000']
    for a,b in zip(rows_by_key['0_0.000000'],old):
        replay_checks.append(a['valid']==b['valid'] and (not a['valid'] or np.allclose(a['planned'],b['planned'],rtol=0,atol=1e-10)))
    xy=np.array([[c['x'],c['y']] for c in cycles]);progress=np.r_[0,np.cumsum(np.linalg.norm(np.diff(xy,axis=0),axis=1))]
    for key,rows in rows_by_key.items():
        ds=[];hd=[]
        for i,r in enumerate(rows):
            if not r['valid'] or not r['used_previous'] or (tag.startswith('S') and not 35<=progress[i]<=110):continue
            a=np.array(r['planned']);b=np.array(r['reference'])
            if len(a)<4:continue
            ds.append(max(np.linalg.norm(a[1:3]-b[1:3],axis=1)))
            u=a[1]-a[0];v=b[1]-b[0];hd.append(abs(np.degrees(np.arctan2(np.cross(v,u),v@u))))
        replay[tag+'/'+key]=dict(failures=sum(not r['valid'] for r in rows),near_p95_m=float(np.quantile(ds,.95)),heading_change_p95_deg=float(np.quantile(hd,.95)))
        print(tag,key,replay[tag+'/'+key])
assert len(checks)==56 and all(checks)
assert len(replay_checks)==114 and all(replay_checks)
assert sum(r['total'] for r in summary.values())==224
result=dict(closed_loop=summary,replay=replay,baseline_closed_loop_matches=len(checks),baseline_replay_cycles_match=len(replay_checks),closed_loop_runs=224)
(here/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
print('Baseline verified: 56 closed-loop conditions, 114 replay cycles. Total 224 closed-loop runs.')
