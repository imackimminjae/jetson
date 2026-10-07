from pathlib import Path
import json,hashlib,pickle,sqlite3
import numpy as np
here=Path(__file__).resolve().parent;root=here.parents[1];old=here.parent/'soft_cap12_trial'
out=json.loads((here/'validation.json').read_text())
def error(a,b):
 assert len(a)==len(b)
 errors=[]
 for x,y in zip(a,b):
  assert x['valid']==y['valid']
  if x['valid']:errors.append(float(np.max(np.linalg.norm(np.array(x['planned'])-y['planned'],axis=1))))
 return max(errors,default=0)
for tag in ['050627','050536','014143','014303','045314','044738']:
 a=json.loads((here/f'candidate_{tag}_current.json').read_text())['baseline'];b=json.loads((old/f'result_{tag}_current.json').read_text())['0_0.000000'];out[tag+'/current_vs_previous_max_error_m']=error(a,b);assert out[tag+'/current_vs_previous_max_error_m']<1e-10
for tag in ['050627','050536','055105']:
 D=json.loads((here/f'diagnostic_{tag}_recorded.json').read_text());C=json.loads((here/f'replay_{tag}.json').read_text())['cycles']
 for k,rows in D.items():
  if k.endswith('_B1'):
   assert all(r['B']==c['rec_B'] for r,c in zip(rows,C)),k
 out[tag+'/fixed_B_exact']=True
# Verify the newly discovered cached branch events and lower traces against raw read-only bag.
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
folder=root/'drive_debug_20261006_055105_light';cache=pickle.load((folder/'analysis/decoded.pkl').open('rb'));raw={k:[] for k in ['/debug/upper_branch_event','/debug/lower_mpc_trace']}
for p in (folder/'bag').glob('*.db3'):
 con=sqlite3.connect('file:'+str(p)+'?mode=ro',uri=True)
 for name,kind,t,blob in con.execute('select topics.name,topics.type,messages.timestamp,messages.data from messages join topics on topics.id=messages.topic_id where topics.name in (?,?) order by messages.timestamp',tuple(raw)):
  m=deserialize_message(blob,get_message(kind));v=json.loads(m.data) if kind=='std_msgs/msg/String' else np.array(m.data);raw[name].append((t/1e9,v))
 con.close()
for name,rows in raw.items():
 assert len(rows)==len(cache[name]),name
 for (t,v),r in zip(rows,cache[name]):
  assert t==r['t']
  if isinstance(v,dict):assert v==r['data']
  else:np.testing.assert_array_equal(v,r['data'])
out['055105/raw_bag_cache_verified']={k:len(v) for k,v in raw.items()}
manifest={}
for p in list(here.glob('replay_*.json'))+list(here.glob('replay_*_grids.bin'))+list(here.glob('config_*.yaml'))+[folder/'analysis/decoded.pkl',folder/'config_snapshot.yaml']+list((folder/'bag').glob('*.db3')):
 manifest[str(p.resolve())]=hashlib.sha256(p.read_bytes()).hexdigest()
for n in json.loads((here/'baseline/sha256.json').read_text()):
 expected=json.loads((here/'baseline/sha256.json').read_text())[n];assert hashlib.sha256((root/n).read_bytes()).hexdigest()==expected
out['production_seven_files_unchanged']=True
(here/'validation.json').write_text(json.dumps(out,indent=2));(here/'inputs_sha256.json').write_text(json.dumps(manifest,indent=2));print(json.dumps(out,indent=2))
