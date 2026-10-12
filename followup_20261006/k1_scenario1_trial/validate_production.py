from pathlib import Path
import os,subprocess,json,numpy as np
H=Path(__file__).resolve().parent
env=dict(os.environ,ROS_DOMAIN_ID='177',ROS_LOCALHOST_ONLY='1',ROS_LOG_DIR=str(H/'ros_logs'))
out=[]
for m in json.loads((H/'manifest.json').read_text()):
 tag,kind=m['tag'],m['config']
 specs=[s for s in json.loads((H/f'specs_{tag}_{kind}.json').read_text()) if s['skip_k1_mode']!=2]
 sp=H/f'production_specs_{tag}_{kind}.json';sp.write_text(json.dumps(specs))
 op=H/f'production_result_{tag}_{kind}.json'
 with (H/f'production_result_{tag}_{kind}.log').open('w') as log:
  subprocess.run([str(H.parent/'k1_front_clip_skip_trial/production_replay'),str(H/f'input_{tag}.json'),str(H/f'grids_{tag}.bin'),str(H/f'config_{tag}_{kind}.yaml'),str(op),str(sp)],env=env,stdout=log,stderr=subprocess.STDOUT,check=True)
 expected=json.loads((H/f'result_{tag}_{kind}.json').read_text());actual=json.loads(op.read_text())
 for name,rs in actual.items():
  baseline=expected[name];assert len(rs)==len(baseline);err=0.
  for i,(a,b) in enumerate(zip(rs,baseline)):
   assert a['valid']==b['valid'],(tag,kind,name,i,'validity')
   assert a['selected']==b['selected'],(tag,kind,name,i,'selection')
   assert a['k1_skip_mask']==b['k1_skip_mask'],(tag,kind,name,i,'mask')
   if a['valid']:
    err=max(err,float(np.max(np.abs(np.array(a['planned'])-b['planned']))))
  assert err<1e-6,(tag,kind,name,err)
  out.append(dict(tag=tag,config=kind,mode=name,cycles=len(rs),max_error_m=err))
 print('Production matched',tag,kind,flush=True)
(H/'production_validation.json').write_text(json.dumps(out,indent=2))
print('Matched solves',sum(r['cycles'] for r in out),'max error',max(r['max_error_m'] for r in out))
