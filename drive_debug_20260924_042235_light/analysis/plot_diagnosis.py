from pathlib import Path
import json,pickle,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
root=Path(__file__).resolve().parent;d=pickle.load(open(root/'decoded.pkl','rb'))
a=np.array([x['data'] for x in d['/debug/lower_mpc_trace']]);tr=[x['data'] for x in d['/debug/upper_miqp_trace']];ev=[x['data'] for x in d['/debug/upper_branch_event']];snaps=json.loads((root/'snapshot_comparison.json').read_text());zero=next(x[0] for x in tr if x[14]>=1)
fig,axs=plt.subplots(3,1,figsize=(11,10))
for mode,title in [(0,'Published first-segment heading'),(1,'Same snapshot, turn cost = 0')]:
 r=[x for x in snaps if x['mode']==mode];axs[0].plot([x['t']-zero for x in r],[x['first_heading_deg'] for x in r],label=title)
axs[0].plot(a[:,0]-zero,np.degrees(a[:,3]),label='Measured body yaw',color='black',alpha=.65)
axs[0].axvline(0,color='grey',linestyle=':');axs[0].axvline(2,color='grey',linestyle=':');axs[0].set_xlim(-2,9);axs[0].set_ylabel('World heading (deg)');axs[0].set_title('04:23:10.167: branch k=5; +2 seconds: cost window reaches k=1');axs[0].legend()
for folder,label in [('drive_debug_20260924_035423_light','Previous drive'),('drive_debug_20260924_042235_light','Latest drive')]:
 z=pickle.load(open(root.parents[1]/folder/'analysis/decoded.pkl','rb'));b=np.array([x['data'] for x in z['/debug/lower_mpc_trace']]);start=next(x['data'][0] for x in z['/debug/upper_miqp_trace'] if x['data'][14]>=1)
 axs[1].plot(b[:,0]-start,np.degrees(b[:,8]),label=label)
axs[1].set_xlim(-2,9);axs[1].set_ylabel('Applied steering (deg)');axs[1].set_xlabel('Seconds from first detected branch');axs[1].legend()
route=d['/navigation/global_path'][0]['xy'];axs[2].plot(route[:,0],route[:,1],'k--',alpha=.4,label='Global waypoint polyline')
mask=(a[:,0]>=zero-1)&(a[:,0]<=zero+9);axs[2].plot(a[mask,1],a[mask,2],'k',label='Driven trajectory')
for dt,color in [(0,'tab:orange'),(1,'tab:red'),(2,'tab:blue')]:
 e=min(ev,key=lambda e:abs(e['t_emit']-zero-dt));p=np.array(e['planned_world']);axs[2].plot(p[:,0],p[:,1],'.-',color=color,label=f'Plan at branch +{dt}s');axs[2].scatter(p[0,0],p[0,1],s=55,color=color)
axs[2].set_xlim(98,124);axs[2].set_ylim(-59,-20);axs[2].set_aspect('equal',adjustable='box');axs[2].set_xlabel('World x (m)');axs[2].set_ylabel('World y (m)');axs[2].legend(loc='upper left',bbox_to_anchor=(1,1))
for ax in axs:ax.grid(alpha=.25)
fig.tight_layout();fig.savefig(root/'latest_branch_diagnosis.png',dpi=150)
