from pathlib import Path
import json,gzip,os,subprocess,sys
H=Path(__file__).resolve().parent;R=H.parents[1]
which=sys.argv[1] if len(sys.argv)>1 else 'batch'
if which=='batch':
 manifest=json.loads((H/'batch_manifest.json').read_text())
 specs=[]
 for edge,label in [(0,'base'),(1,'edge')]:
  for fr,fb,name in [(True,True,'fixed'),(False,True,'B_fixed'),(True,False,'ref_fixed'),(False,False,'carried')]:
   specs.append(dict(name=label+'_'+name,edge_mode=edge,fixed_ref=fr,fixed_B=fb,fixed_wp=True,guard=True))
 specs += [dict(name='hold_update',fixed_ref=True,fixed_wp=True,hold_update=True,guard=True),dict(name='guard_off_fixed',fixed_ref=True,fixed_B=True,fixed_wp=True,guard=False),dict(name='guard_off_carried',fixed_wp=True,guard=False),dict(name='clip_margin_fixed',fixed_ref=True,fixed_B=True,fixed_wp=True,guard=True,edge_mode=2),dict(name='clip_raw_fixed',fixed_ref=True,fixed_B=True,fixed_wp=True,guard=True,edge_mode=3)]
else:
 P=H.parent/'side_boundary_tail_trial';manifest=[dict(tag=tag,input=str(P/f'input_{tag}.json'),grids=str(P/f'grids_{tag}.bin'),config=str(P/f'config_{tag}.yaml')) for tag in json.loads((P/'tags.json').read_text())]
 # Add earlier failure and success cases without copying their map binaries.
 for tag in ['050536','014143','014303','045314','044738']:
  P=R/'followup_20261006/reference_B_separation'
  manifest.append(dict(tag='20261006_'+tag,input=str(P/f'replay_{tag}.json'),grids=str(P/f'replay_{tag}_grids.bin'),config=str(P/f'config_{tag}.yaml')))
 specs=[dict(name=label+'_'+mode,edge_mode=edge,fixed_ref=mode=='fixed',fixed_B=mode=='fixed',fixed_wp=True,guard=guard) for label,edge,guard in [('recorded',0,False),('base',0,True),('edge',1,True)] for mode in ['fixed','carried']]
 # above list deliberately assembled below for clarity
specpath=H/f'specs_{which}.json';specpath.write_text(json.dumps(specs,indent=2))
env=dict(os.environ,ROS_DOMAIN_ID='177',ROS_LOCALHOST_ONLY='1',ROS_LOG_DIR=str(H/'ros_logs'))
for a in manifest:
 out=H/f'result_{a["tag"]}.json';g=Path(a['grids']);data=gzip.decompress(g.read_bytes()) if g.suffix=='.gz' else None
 with (H/f'result_{a["tag"]}.log').open('w') as log:
  subprocess.run([str(H/'replay'),a['input'],'/dev/stdin' if data is not None else str(g),a['config'],str(out),str(specpath)],input=data,env=env,stdout=log,stderr=subprocess.STDOUT,check=True)
 with gzip.open(str(out)+'.gz','wb') as f:f.write(out.read_bytes())
 out.unlink();print('Finished',a['tag'],flush=True)
(H/f'manifest_{which}.json').write_text(json.dumps(manifest,indent=2))
