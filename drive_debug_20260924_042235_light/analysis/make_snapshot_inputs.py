from pathlib import Path
import pickle,numpy as np
root=Path(__file__).resolve().parent;d=pickle.load(open(root/'decoded.pkl','rb'))
evs=[x['data'] for x in d['/debug/upper_branch_event']];tr=[x['data'] for x in d['/debug/upper_miqp_trace']]
route=d['/navigation/global_path'][0]['xy'];lines=[str(len(evs))]
for e,x in zip(evs,tr):
 assert abs(x[0]-e['t_emit'])<.1
 n=int(x[7]);yaw=x[3];xy=np.array([e['x'],e['y']]);R=np.array([[np.cos(yaw),-np.sin(yaw)],[np.sin(yaw),np.cos(yaw)]])
 tobody=lambda p:(np.array(p)-xy)@R
 vals=[x[0],yaw,n,6.0,*x[11:14],*tobody(route[e['wp0']]),*tobody(route[e['wp1']]),*tobody(e['preview_world']).ravel(),*x[18:18+n+1],*e['candidate_counts']]
 for k in range(1,n+1):
  for j in range(e['candidate_counts'][k]):
   inter=next(r for r in e['intervals_world'] if r[0]==k and r[1]==j)
   vals.extend(tobody(inter[2:4]));vals.extend(tobody(inter[4:6]))
 lines.append(' '.join(map(str,vals)))
(root/'snapshot_inputs.txt').write_text('\n'.join(lines)+'\n')
