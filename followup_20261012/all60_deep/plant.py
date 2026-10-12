import pickle,json,gzip,numpy as np
def fit(od,lo,windows,label):
    best=[]
    for tau in [0.3,0.5,0.74,0.9,1.2,1.6]:
        for dly in [0.0,0.1,0.2,0.3,0.5]:
            num=0;den=0;errs=[]
            for s,e in windows:
                m=(od[:,0]>=s+5)&(od[:,0]<e);O=od[m]
                if len(O)<50: continue
                t=np.arange(O[0,0],O[-1,0],0.02);yaw=np.interp(t,O[:,0],np.unwrap(O[:,3]));v=np.interp(t,O[:,0],np.hypot(O[:,4],O[:,5]))
                yr=np.convolve(np.gradient(yaw,t),np.ones(15)/15,'same');dimp=np.arctan(yr*2.8/np.maximum(v,1))
                L=lo[(lo[:,0]>=s)&(lo[:,0]<e)];idx=np.searchsorted(L[:,0],t-dly)-1;cmd=np.where(idx>=0,L[np.clip(idx,0,None),26],0)
                d=np.zeros_like(t);al=1-np.exp(-0.02/tau)
                for i in range(1,len(t)): d[i]=d[i-1]+al*(cmd[i]-d[i-1])
                sel=v>3;num+=np.sum(d[sel]*dimp[sel]);den+=np.sum(d[sel]**2);errs.append((d[sel],dimp[sel]))
            g=num/den;e2=np.sqrt(np.mean(np.concatenate([(g*a-b)**2 for a,b in errs])));e1=np.sqrt(np.mean(np.concatenate([(a-b)**2 for a,b in errs])))
            best.append((e2,tau,dly,g,e1))
    best.sort();e2,tau,dly,g,e1=best[0]
    print(f"{label}: best tau {tau} s, delay {dly} s, effective gain {g:.2f} (1.0 = command fully realised); rms {np.degrees(e2):.2f} deg (gain=1: {np.degrees(e1):.2f})")
D=pickle.load(open('decoded.pkl','rb'));A=pickle.load(open('attempts.pkl','rb'))
od=np.array(D['/motive/vehicle/odom_map']);lo=np.array([v for t,v in D['/debug/lower_mpc_trace']])
fit(od,lo,[(a['start'],a['end']) for a in A['att']],'10-09 all 60')
for r in (1,3,5,9):
    fit(od,lo,[(a['start'],a['end']) for a in A['att'] if a['route']==r],f'10-09 S{r}')
for n,lab in [(3,'10-08 S3'),(1,'10-08 S1')]:
    H=f'/home/imac/ros2_ws/followup_20261008/batches_182344_183952_185426_review/scenario{n}/';T=json.load(gzip.open(H+'telemetry.json.gz','rt'));S=json.load(open(H+'summary.json'))
    od2=np.array([[t]+v[:5] for t,v in T['data']['/motive/vehicle/odom_map']],dtype=float);lo2=np.array([v for _,v in T['data']['/debug/lower_mpc_trace']],dtype=float)
    fit(od2,lo2,[(a['start'],a['end']) for a in S['attempts']],lab)
