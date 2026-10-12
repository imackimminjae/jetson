from pathlib import Path
import json,numpy as np
from datetime import datetime
from zoneinfo import ZoneInfo
H=Path(__file__).resolve().parent
def choice(r):
 if not r['valid']:return 'failed'
 q=np.array(r['planned'][-1])-[121,-7]
 if np.linalg.norm(q)<8 or q[1]<-4:return 'before'
 return 'E' if np.degrees(np.arctan2(q[1],q[0]))<42 else 'N'
clock=lambda t:datetime.fromtimestamp(t,ZoneInfo('Asia/Seoul')).strftime('%H:%M:%S.%f')[:-3]
summary=[];witness=[];total=0
for m in json.loads((H/'manifest.json').read_text()):
 tag,kind=m['tag'],m['config'];origin=m['effective_origin']
 cs=json.loads((H/f'input_{tag}.json').read_text())['cycles']
 grids=np.fromfile(H/f'grids_{tag}.bin',dtype=np.int8).reshape(-1,128,128)
 ds=json.loads((H/f'result_{tag}_{kind}.json').read_text())
 for name,rs in ds.items():
  total+=len(rs);base=ds[name.split('_')[0]+'_m0'];maxdelta=0.;changed=0;reg=[];improve=[];blocked=[];newblocked=[]
  for i,(c,a,b) in enumerate(zip(cs,base,rs)):
   if a['valid']!=b['valid']:changed+=1
   if a['valid'] and b['valid']:
    if len(a['planned'])!=len(b['planned']):changed+=1
    else:
     delta=float(np.linalg.norm(np.array(a['planned'])-b['planned'],axis=1).max());maxdelta=max(maxdelta,delta)
     changed+=delta>1e-6
   focus=110<c['x']<132 and -22<c['y']<6
   ca,cb=choice(a),choice(b)
   if focus:
    if ca=='E' and cb=='N':reg.append(i)
    if ca=='N' and cb=='E':improve.append(i)
    witness.append(dict(tag=tag,config=kind,mode=name,i=i,time=clock(c['t']),pose=[c['x'],c['y']],base=ca,variant=cb,base_end=a.get('planned',[None])[-1],end=b.get('planned',[None])[-1]))
   def cell(r):
    if not r['valid'] or len(r['planned'])<2:return None
    yaw=c['yaw'];R=np.array([[np.cos(yaw),-np.sin(yaw)],[np.sin(yaw),np.cos(yaw)]])
    p=(np.array(r['planned'][1])-[c['x'],c['y']])@R
    x,y=np.floor((p-[origin,-20])/.3125).astype(int)
    return int(grids[c['grid_index'],y,x]) if 0<=x<128 and 0<=y<128 else -1
   if b['valid'] and cell(b)>=1:
    blocked.append(i)
    if cell(a)==0:newblocked.append(i)
  row=dict(tag=tag,config=kind,mode=name,cycles=len(rs),valid=sum(r['valid'] for r in rs),skip_cycles=sum(any(r['k1_skip_mask']) for r in rs),changed_cycles=changed,max_plan_delta_m=maxdelta,correct_to_wrong_indices=reg,wrong_to_correct_indices=improve,k1_blocked_indices=blocked,k1_new_blocked_vs_valid_free_baseline=newblocked)
  if kind=='recorded' and name=='fixed_m0':
   errors=[];matched=0
   for c,r in zip(cs,rs):
    matched+=c['rec_valid']==r['valid']
    if c['rec_valid'] and r['valid'] and len(c['rec_planned'])==len(r['planned']):errors.append(float(np.linalg.norm(np.array(c['rec_planned'])-r['planned'],axis=1).max()))
   row['recorded_fidelity']=dict(validity_matches=matched,comparable_plans=len(errors),plans_within_1mm=sum(e<.001 for e in errors),max_plan_error_m=max(errors,default=0))
  summary.append(row)
(H/'summary.json').write_text(json.dumps(dict(total_solves=total,results=summary),indent=2))
(H/'branch_decisions.json').write_text(json.dumps(witness,indent=2))
print('Total solves',total)
for r in summary:
 if r['mode'].startswith('carried'):print(r['tag'],r['config'],r['mode'],'valid',r['valid'],'changed',r['changed_cycles'],'E->N',r['correct_to_wrong_indices'],'N->E',r['wrong_to_correct_indices'],'newblocked',r['k1_new_blocked_vs_valid_free_baseline'])
