import pickle,json,gzip,numpy as np,matplotlib;matplotlib.use('Agg');import matplotlib.pyplot as plt
from matplotlib import font_manager
font_manager.fontManager.addfont('/usr/share/fonts/truetype/nanum/NanumGothic.ttf');plt.rcParams.update({'font.family':'NanumGothic','axes.unicode_minus':False})
D=pickle.load(open('decoded.pkl','rb'));A=pickle.load(open('attempts.pkl','rb'))
od=np.array(D['/motive/vehicle/odom_map']);cs=np.array([v for t,v in D['/debug/upper_interval_centering_state']]);lo=np.array([v for t,v in D['/debug/lower_mpc_trace']])
H='/home/imac/ros2_ws/followup_20261008/batches_182344_183952_185426_review/scenario3/';T=json.load(gzip.open(H+'telemetry.json.gz','rt'));S=json.load(open(H+'summary.json'))
od2=np.array([[t]+v[:5] for t,v in T['data']['/motive/vehicle/odom_map']],dtype=float);cs2=np.array([v for _,v in T['data']['/debug/upper_interval_centering_state']]);lo2=np.array([v for _,v in T['data']['/debug/lower_mpc_trace']],dtype=float)
rt=np.array(S['route'])
fig,axs=plt.subplots(3,1,figsize=(14,15))
BC='/home/imac/ros2_ws/followup_20261009/s1_block_compare/regress/s3/';m=json.load(open(BC+'static_map.json'));B=np.fromfile(BC+'static_map.bin',dtype=np.int8).reshape(m['height'],m['width'])
axs[0].imshow(B==0,origin='lower',extent=(m['origin'][0],m['origin'][0]+m['width']*m['res'],m['origin'][1],m['origin'][1]+m['height']*m['res']),cmap='Greys_r',alpha=.5)
axs[0].plot(rt[:,0],rt[:,1],'k--o',ms=4)
def seg(odx,s,e):
    mm=(odx[:,0]>=s)&(odx[:,0]<e);O=odx[mm];sd=np.concatenate([[0],np.cumsum(np.hypot(np.diff(O[:,1]),np.diff(O[:,2])))]);return O,sd
for (odx,csx,lox,atts,col,lab) in [(od2,cs2,lo2,[(a['start'],a['end']) for a in S['attempts']],'teal','10-08 (10/10 성공)'),(od,cs,lo,[(a['start'],a['end']) for a in A['att'] if a['route']==3],'crimson','10-09 (0/5)')]:
    for k,(s,e) in enumerate(atts):
        O,sd=seg(odx,s,e);axs[0].plot(O[:,1],O[:,2],c=col,lw=1,alpha=.8,label=lab if k==0 else None)
        c=csx[(csx[:,0]>=s)&(csx[:,0]<e)];axs[1].plot(np.interp(c[:,0],O[:,0],sd),c[:,2],c=col,lw=1,drawstyle='steps-post',alpha=.8,label=lab if k==0 else None)
        L=lox[(lox[:,0]>=s)&(lox[:,0]<e)];axs[2].plot(np.interp(L[:,0],O[:,0],sd),np.degrees(L[:,26]),c=col,lw=.8,alpha=.8,label=lab if k==0 else None)
axs[0].set_xlim(180,300);axs[0].set_ylim(120,225);axs[0].set_aspect('equal');axs[0].legend();axs[0].grid(alpha=.3);axs[0].set_title('시나리오3 궤적: 10-08 vs 10-09')
for a,yl in zip(axs[1:],['B (m)','출력 조향 명령 (도)']): a.set_xlabel('주행 거리 (m)');a.set_ylabel(yl);a.grid(alpha=.3);a.legend()
fig.savefig('s3_1008_vs_1009.png',dpi=65,bbox_inches='tight')
