from pathlib import Path
import json,gzip,csv,hashlib,os
import numpy as np
H=Path(__file__).resolve().parent;R=H.parents[1]
load=lambda p:json.load(gzip.open(p,'rt')) if str(p).endswith('.gz') else json.loads(Path(p).read_text())
def maxdist(p,q):
    if len(p)!=len(q):return 1e9
    return float(np.max(np.linalg.norm(np.array(p)-q,axis=1))) if len(p) else 0.
def lat(pt,path):
    a=np.array(path);p=np.array(pt);v=np.diff(a,axis=0);ll=np.sum(v*v,axis=1);u=np.clip(np.sum((p-a[:-1])*v,axis=1)/np.maximum(ll,1e-12),0,1);q=a[:-1]+u[:,None]*v;j=np.argmin(np.linalg.norm(p-q,axis=1));return float((v[j,0]*(p-q[j])[1]-v[j,1]*(p-q[j])[0])/max(np.sqrt(ll[j]),1e-9))
validation=[];comparisons=[];critical=[];detailed=[]
manifest=[dict(tag=f'latest_sc1_{n}',input=str(H/f'input_{n}.json'),result=str(H/f'upper_{n}.json.gz'),refine=str(H/f'refine_sc1_{n}.json.gz')) for n in range(1,11)]
manifest+=[dict(**a,result=str(H/(a['tag']+'.json.gz')),refine=str(H/('refine_'+a['tag']+'.json.gz'))) for a in load(H/'regression_manifest.json')]
for a in manifest:
    cs=load(a['input'])['cycles'];rr=load(a['result']);baseline=rr['fixed'];refine=Path(a['refine'])
    if refine.exists():rr.update(load(refine))
    diffs=[];mismatch=[]
    for j,(c,b) in enumerate(zip(cs,baseline)):
        if bool(c['rec_valid'])!=b['valid']:mismatch.append(j)
        if c['rec_valid'] and b['valid']:
            diffs.append(dict(cycle=j,error=maxdist(c.get('rec_published',c['rec_planned']),b['published']),x=c['x'],y=c['y']))
    validation.append(dict(tag=a['tag'],cycles=len(cs),valid_mismatch=mismatch,valid_comparable=len(diffs),within_1mm=sum(v['error']<=.001 for v in diffs),max_error=max(v['error'] for v in diffs),inexact=[v for v in diffs if v['error']>.001]))
    for mode,vals in rr.items():
        new=[];resolved=[];choices=[]
        for j,(c,b,z) in enumerate(zip(cs,baseline,vals)):
            if not c.get('running',True):continue
            if b['valid'] and not z['valid']:new.append(j)
            if not b['valid'] and z['valid']:resolved.append(j)
            if b['valid'] and z['valid'] and b['selected']!=z['selected']:
                choices.append(j)
                detailed.append(dict(tag=a['tag'],mode=mode,cycle=j,x=c['x'],y=c['y'],fixed_end=b['published'][-1],variant_end=z['published'][-1],fixed_selected=b['selected'],variant_selected=z['selected']))
        comparisons.append(dict(tag=a['tag'],mode=mode,cycles=sum(c.get('running',True) for c in cs),fixed_fail=sum(not b['valid'] for c,b in zip(cs,baseline) if c.get('running',True)),variant_fail=sum(not z['valid'] for c,z in zip(cs,vals) if c.get('running',True)),new_fail=len(new),resolved=len(resolved),changed_corridor=len(choices),new_fail_cycles=new,resolved_cycles=resolved,changed_corridor_cycles=choices))
    if not a['tag'].startswith('latest'):continue
    n=int(a['tag'].split('_')[-1]);low=load(H/f'lower_{n}.json.gz');err=[abs(z['recorded']['delta']-z['recorded_delta']) for z in low];out_err=[abs(z['recorded']['output']-z['recorded_output']) for z in low]
    validation[-1].update(lower_samples=len(low),lower_valid=sum(z['recorded']['valid'] for z in low),lower_delta_max_error_rad=max(err),lower_output_max_error_rad=max(out_err),lower_matches_1e_7=sum(e<=1e-7 for e in err))
    eligible=[i for i,c in enumerate(cs) if -24<c['y']<-21 and c['x']<120];j=min(eligible,key=lambda i:abs(cs[i]['y']+22.5));c=cs[j]
    prev=next(cs[k]['rec_published'] for k in range(j-1,-1,-1) if cs[k]['rec_valid'])
    q=[z for z in low if z['cycle']==j];peak=max(q,key=lambda z:z['fixed']['output'])
    if (H/f'ablation_{n}.json.gz').exists():rr.update(load(H/f'ablation_{n}.json.gz'))
    item=dict(attempt=n,cycle=j,t=c['t'],xy=[c['x'],c['y']],baseline_error=maxdist(c['rec_published'],baseline[j]['published']),lower_t=peak['t'],lower_xy=[peak['x'],peak['y']],modes={})
    for mode,vals in rr.items():
        z=vals[j];v=dict(valid=z['valid'],B=z['B'],k1_shift=lat(z['published'][1],prev) if z['valid'] else None,k1_vs_preview=lat(z['published'][1],c['rec_preview']) if z['valid'] else None,reference_max_change=maxdist(z['reference'],baseline[j]['reference']))
        if mode in peak:v['output_deg']=float(np.degrees(peak[mode]['output']))
        item['modes'][mode]=v
    critical.append(item)
result=dict(validation=validation,comparisons=comparisons,critical=critical,corridor_changes=detailed)
(H/'summary.json').write_text(json.dumps(result,indent=2))
with (H/'regression.csv').open('w') as f:
    keys=[k for k in comparisons[0] if not k.endswith('_cycles')];w=csv.DictWriter(f,fieldnames=keys);w.writeheader();w.writerows({k:r[k] for k in keys} for r in comparisons)
before=load(H/'operating_sha256_before.json');changed=[p for p,h in before.items() if not Path(p).exists() or hashlib.sha256(Path(p).read_bytes()).hexdigest()!=h]
(H/'operating_unchanged.json').write_text(json.dumps(dict(files_checked=len(before),changed=changed),indent=2))
print('baseline',sum(v['cycles'] for v in validation),'valid mismatches',sum(len(v['valid_mismatch']) for v in validation),'ops changed',changed)
print('latest baseline',[(v['tag'],v['within_1mm'],v['valid_comparable'],v.get('lower_matches_1e_7'),v.get('lower_samples')) for v in validation[:10]])
for q in critical:
    if q['attempt'] in [1,3,5,6,8,10]:print('critical',q['attempt'],{m:{k:round(v,3) if isinstance(v,float) else v for k,v in r.items()} for m,r in q['modes'].items() if m in ['fixed','B7_4','B14','soft_all','soft_k1','relax_only','soft_k15','soft_k12345']})
print('refine regressions',[(q['tag'],q['fixed_fail'],q['variant_fail'],q['new_fail'],q['changed_corridor']) for q in comparisons if q['mode']=='relax_only'])
assert not changed
assert all(not v['valid_mismatch'] for v in validation)
assert all(v['lower_matches_1e_7']==v['lower_samples'] for v in validation[:10])
assert all(q['baseline_error']<1e-7 for q in critical)
