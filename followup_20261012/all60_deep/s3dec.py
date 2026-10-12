import pickle,json,gzip,numpy as np
D=pickle.load(open('decoded.pkl','rb'));A=pickle.load(open('attempts.pkl','rb'))
es=[e for t,e in D['/debug/upper_branch_event']];cs=np.array([v for t,v in D['/debug/upper_interval_centering_state']])
def show(es,cs,s,e,label):
    print('==',label)
    for ev in es:
        if not (s<=ev['t_start']<e) or not (232<ev['x']<262 and ev['y']<150): continue
        P=np.array(ev['published_world']) if ev['published_world'] else None
        hd=np.degrees(np.arctan2(*(P[-1]-P[-2])[::-1])) if P is not None and len(P)>1 else float('nan')
        j=np.argmin(abs(cs[:,0]-ev['t_start']))
        print(f"  ({ev['x']:.1f},{ev['y']:.1f}) yaw{np.degrees(ev['yaw_rad']):+4.0f} wp{ev['wp0']},{ev['wp1']} N{ev['horizon']} B{cs[j,2]:5.1f} br{ev['branch_step']:2d} cnt{ev['candidate_counts']} sel{ev['selected_corridors']} endhd{hd:+4.0f} {'ok' if ev['valid_plan'] else ev['status'][:30]}")
for a in A['att']:
    if a['route']==3 and a['rep'] in (1,4): show(es,cs,a['start'],a['end'],f"10-09 S3 A{a['rep']} {a['outcome']}")
H='/home/imac/ros2_ws/followup_20261008/batches_182344_183952_185426_review/scenario3/'
E2=json.load(gzip.open(H+'events.json.gz','rt'));S=json.load(open(H+'summary.json'));T=json.load(gzip.open(H+'telemetry.json.gz','rt'));cs2=np.array([v for _,v in T['data']['/debug/upper_interval_centering_state']])
for a in S['attempts'][:2]: show(E2,cs2,a['start'],a['end'],f"10-08 S3 #{a['attempt']} {a['outcome']}")
