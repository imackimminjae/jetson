import pickle,json,csv,glob,numpy as np
if __name__=="__main__" or True:
  pass
D=pickle.load(open('decoded.pkl','rb'))
rows=list(csv.DictReader(open(glob.glob('/home/imac/ros2_ws/drive_batches/20261009_070714_all_d8c3cf/results/*/summary.csv')[0])))
st=D['/knu_batch/status'];ph=[];prev=None
for t,s in st:
    key=(s.get('index'),s.get('phase'),s.get('sequence'))
    if key!=prev: ph.append(dict(t=t,**s));prev=key
od=np.array(D['/motive/vehicle/odom_map'])
routes={}
for n in range(1,13):
    f=f'/home/imac/ros2_ws/drive_batches/20261009_070714_all_d8c3cf/routes/scenario{n}.csv'
    try: routes[n]=np.array([[float(r['x']),float(r['y'])] for r in csv.DictReader(open(f))])
    except FileNotFoundError: routes[n]=np.array([[float(r['x']),float(r['y'])] for r in csv.DictReader(open(f'/home/imac/ros2_ws/src/virtual_control/data/knu_routes/scenario{n}.csv'))])
def dist_to_route(P,R):
    a=R[:-1];b=R[1:];v=b-a;L2=(v*v).sum(1);out=[]
    for p in P:
        u=np.clip(((p-a)*v).sum(1)/L2,0,1);q=a+u[:,None]*v;out.append(np.min(np.linalg.norm(q-p,axis=1)))
    return np.array(out)
att=[]
for i,row in enumerate(rows,1):
    ps=[p for p in ph if p.get('index')==i]
    start=next((p['phase_stamp_sec'] for p in ps if p['phase']=='running'),None);end=next((p['phase_stamp_sec'] for p in ps if p['phase']=='coasting'),None)
    n=int(row['route'][8:]);m=(od[:,0]>=start)&(od[:,0]<end);O=od[m]
    d=dist_to_route(O[:,1:3],routes[n]);k=np.argmax(d>5) if (d>5).any() else None
    att.append(dict(i=i,route=n,rep=sum(1 for a in att if a['route']==n)+1,outcome=row['outcome'],progress=float(row['progress_ratio']),start=start,end=end,
        div_xy=(O[k,1],O[k,2]) if k is not None else None,div_t=(O[k,0]-start) if k is not None else None))
    a=att[-1];print(f"#{i:2d} S{n:<2d} A{a['rep']} {a['outcome']:12s} prog {a['progress']:.2f} " + (f"first >5 m off route at ({a['div_xy'][0]:.0f},{a['div_xy'][1]:.0f}) t={a['div_t']:.1f}s" if a['div_xy'] else ''))
pickle.dump(dict(att=att,routes=routes,ph=ph),open('attempts.pkl','wb'))
