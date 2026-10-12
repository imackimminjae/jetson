from pathlib import Path
import json,gzip,os,subprocess,bisect
import numpy as np,yaml
H=Path(__file__).resolve().parent;R=H.parents[1]
os.sched_setaffinity(0,{min(os.sched_getaffinity(0))})
S=R/'followup_20261008/batches_182344_183952_185426_review/scenario1'
events=json.load(gzip.open(S/'events.json.gz','rt'));bytime={e['t_start']:e for e in events}
lower=np.array(json.load(gzip.open(H/'replay_telemetry.json.gz','rt'))['lower'])
cfg=yaml.safe_load((R/'drive_batches/20261008_182344_scenario1_0cefb5/effective_config.yaml').read_text())
(H/'lower_config.yaml').write_text(yaml.safe_dump({'lower_tracking_mpc_node':cfg['lower_tracking_mpc_node']}))
env=dict(os.environ,ROS_DOMAIN_ID='177',ROS_LOCALHOST_ONLY='1',ROS_LOG_DIR=str(H/'ros_logs'),OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
def densify(points):
    pts=[np.array(points[0])]
    for p in points[1:]:
        if np.linalg.norm(p-pts[-1])>1e-8:pts.append(np.array(p))
    pts=np.array(pts);arc=np.r_[0,np.cumsum(np.linalg.norm(np.diff(pts,axis=0),axis=1))]
    dense=[];ss=[];q=0.;j=0
    while q<arc[-1]:
        while j+1<len(arc) and arc[j+1]<q:j+=1
        fraction=(q-arc[j])/(arc[j+1]-arc[j]);dense.append(((1-fraction)*pts[j]+fraction*pts[j+1]).tolist());ss.append(q);q+=.6
    if not len(dense) or np.linalg.norm(dense[-1]-pts[-1])>1e-6:dense.append(pts[-1].tolist());ss.append(float(arc[-1]))
    return dict(xy=dense,s=ss)
for n in range(1,11):
    cs=json.loads((H/f'input_{n}.json').read_text())['cycles'];rr=json.load(gzip.open(H/f'upper_{n}.json.gz','rt'))
    refine=H/f'refine_sc1_{n}.json.gz'
    if refine.exists():rr.update(json.load(gzip.open(refine,'rt')))
    paths=[];idx={};valid=[];times=[]
    for j,c in enumerate(cs):
        if not c['rec_valid']:continue
        valid.append(j);times.append(bytime[c['t']]['t_emit']);idx[j]={'recorded':len(paths)};paths.append(densify(c['rec_published']))
        for mode,zs in rr.items():
            z=zs[j]
            # Only direct path substitutions with a valid new plan are compared.
            # Newly rejected upper plans are counted separately, not silently given zero steering.
            if not z['valid']:continue
            idx[j][mode]=len(paths);paths.append(dict(xy=z['dense'],s=z['dense_s']) if 'dense' in z else densify(z['published']))
    samples=[]
    for tr in lower:
        if not cs[0]['t']<=tr[0]<=cs[-1]['t']+.5 or tr[12]!=0 or tr[4]<=1:continue
        if not tr[2]>=-40 or not tr[1]<=160:continue
        i=bisect.bisect_right(times,tr[0])-1
        if i<0:continue
        j=valid[i];samples.append(dict(trace=tr.tolist(),cycle=j,paths=idx[j]))
    ip=H/f'lower_input_{n}.json';ip.write_text(json.dumps(dict(paths=paths,samples=samples)))
    op=H/f'lower_{n}.json'
    with (H/f'lower_{n}.log').open('w') as log:subprocess.run([str(H/'lower_probe'),str(ip),str(op),str(H/'lower_config.yaml')],env=env,stdout=log,stderr=subprocess.STDOUT,check=True)
    for p in [ip,op]:
        with gzip.open(str(p)+'.gz','wb') as f:f.write(p.read_bytes())
        p.unlink()
    print('lower',n,len(samples),flush=True)
