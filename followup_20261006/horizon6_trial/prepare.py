from pathlib import Path
import json,shutil,hashlib,copy,difflib,yaml
h=Path(__file__).resolve().parent;r=h.parents[1];old=h.parent/'reference_B_separation'
assert not (h/'baseline').exists()
manifest=json.loads((old/'baseline/sha256.json').read_text())
for n,v in manifest.items():
 assert hashlib.sha256((r/n).read_bytes()).hexdigest()==v,n
 p=h/'baseline'/n;p.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(r/n,p)
(h/'baseline/sha256.json').write_text(json.dumps(manifest,indent=2));shutil.copy2(old/'restore.py',h/'restore.py')
(h/'specs.json').write_text(json.dumps([dict(name='carried')]))
inputs={}
for tag in ['050627','050536','055105','014143','014303','045314','044738']:
 original=json.loads((old/f'replay_{tag}.json').read_text())
 for horizon in [7,6]:
  for seed in ['recorded','fresh']:
   d=copy.deepcopy(original)
   d['cycles'][0]['rec_preview']=d['cycles'][0]['rec_preview'][:horizon+1] if seed=='recorded' else []
   (h/f'input_{tag}_h{horizon}_{seed}.json').write_text(json.dumps(d))
 for cfg in ['recorded','current']:
  s=(old/f'config_{tag if cfg=="recorded" else "current"}.yaml').read_text();assert s.count('upper_preview_steps: 7')==1
  for horizon in [7,6]:
   n=s.replace('upper_preview_steps: 7',f'upper_preview_steps: {horizon}')
   (h/f'config_{tag}_{cfg}_h{horizon}.yaml').write_text(n)
   a=yaml.safe_load(s);b=yaml.safe_load(n);b['upper_planner_node']['ros__parameters']['upper_preview_steps']=7;assert a==b
 for p in [old/f'replay_{tag}.json',old/f'replay_{tag}_grids.bin',old/f'config_{tag}.yaml']:
  inputs[str(p)]=hashlib.sha256(p.read_bytes()).hexdigest()
for p in [old/'replay',old/'replay.cpp',old/'tracking_control_trial.cpp',old/'tracking_control_trial.o',old/'config_current.yaml']:
 inputs[str(p)]=hashlib.sha256(p.read_bytes()).hexdigest()
(h/'input_source_sha256.json').write_text(json.dumps(inputs,indent=2))
s=(old/'config_current.yaml').read_text();n=s.replace('upper_preview_steps: 7','upper_preview_steps: 6');(h/'candidate.patch').write_text(''.join(difflib.unified_diff(s.splitlines(True),n.splitlines(True),fromfile='current_snapshot.yaml',tofile='offline_horizon6.yaml')))
print('Prepared one-parameter horizon trial; no production edits.')
