import pickle,json,gzip,numpy as np
D=pickle.load(open('decoded.pkl','rb'));A=pickle.load(open('attempts.pkl','rb'))
es=[e for t,e in D['/debug/upper_branch_event']];cs=np.array([v for t,v in D['/debug/upper_interval_centering_state']]);ii=[(t,v) for t,v in D['/debug/upper_interval_info']];iit=np.array([t for t,_ in ii])
H='/home/imac/ros2_ws/followup_20261008/batches_182344_183952_185426_review/scenario3/';T=json.load(gzip.open(H+'telemetry.json.gz','rt'));S=json.load(open(H+'summary.json'));E2=json.load(gzip.open(H+'events.json.gz','rt'))
cs2=np.array([v for _,v in T['data']['/debug/upper_interval_centering_state']]);ii2=[(t,v) for t,v in T['data']['/debug/upper_interval_info']];iit2=np.array([t for t,_ in ii2])
def show(es,cs,ii,iit,s,e,label):
    print('==',label)
    for ev in es:
        if not (s<=ev['t_start']<e) or not (228<ev['x']<246 and ev['y']<152): continue
        j=np.argmin(abs(cs[:,0]-ev['t_start']));r=cs[j];info=np.array(ii[np.argmin(abs(iit-ev['t_start']))][1]).reshape(-1,12)
        W={(int(w[0]),int(w[1])):w for w in ev['intervals_world']}
        k=int(r[5]);ci=int(r[6]);w=W.get((k,ci));loc=f"({(w[2]+w[4])/2:.0f},{(w[3]+w[5])/2:.0f})" if w else ''
        far=[(int(x[0]),int(x[1]),round(x[2],1),int(x[9])) for x in info if x[0]>=4]
        print(f"  veh({ev['x']:.1f},{ev['y']:.1f}) yaw{np.degrees(ev['yaw_rad']):+4.0f} B {r[1]:.1f}->{r[2]:.1f} cand {r[4]:.1f}@k{k} {loc} | far k>=4 (k,i,raw,refin): {far}")
for a in A['att']:
    if a['route']==3 and a['rep'] in (1,2): show(es,cs,ii,iit,a['start'],a['end'],f"10-09 S3 A{a['rep']}")
for a in S['attempts'][:2]: show(E2,cs2,ii2,iit2,a['start'],a['end'],f"10-08 S3 #{a['attempt']}")
