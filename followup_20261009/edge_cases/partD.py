import sys;sys.path.insert(0,'/home/imac/ros2_ws/followup_20261009/edge_cases')
from common import *
# ---- case 18: recorder file rotation gap
G=json.load(open('/home/imac/ros2_ws/followup_20261008/scenario2_causes/rotation_gaps.json'));t0=G[0]['t']
fig,ax=plt.subplots(figsize=(11,4.5));x=[(g['t']-t0)/60 for g in G];y=[g['max_grid_gap'] for g in G]
ax.bar(x,y,width=.25,color=['crimson' if v>1 else 'teal' for v in y]);ax.axhline(2.0,ls='--',c='k');ax.text(0.2,2.05,'배치 runner의 recorder 감시 한계 2 s',fontsize=9)
ax.set_xlabel('배치 시작 후 (분) — 녹화 파일 전환 시점마다');ax.set_ylabel('BEV 수신 최대 공백 (s)');ax.grid(alpha=.3)
ax.set_title('사례 18  녹화 파일 전환 시 토픽 공백: 7번째 전환에서만 2.32 s → recorder heartbeat lost로 배치 중단',fontsize=11)
note(fig,'시나리오2 20261008_171715 · 다른 토픽도 같은 2.3~2.5 s 공백 후 몰려 기록됨 · 원인(저장/DDS 지연) 미확정')
fig.savefig('case18_recorder_gap.png',dpi=90,bbox_inches='tight');plt.close(fig)
# ---- case 20: coasting after completed (scenario1 and 3)
fig,axs=plt.subplots(1,2,figsize=(15,6))
for ax,n,pref,xl,yl in [(axs[0],1,'',(150,215),(-20,15)),(axs[1],3,'regress/s3/',(255,310),(150,215))]:
    H=f'/home/imac/ros2_ws/followup_20261008/batches_182344_183952_185426_review/scenario{n}/';S=json.load(open(H+'summary.json'));tr=json.load(gzip.open(H+'tracks.json.gz','rt'))
    bg(ax,pref);goal=S['route'][-1];ax.add_patch(plt.Circle(goal,8,fill=False,ec='green',lw=1.5,ls='--'));ax.plot(*goal,'g*',ms=14,label='목표 (완료 반경 8 m)')
    d=[]
    for a,t in zip(S['attempts'],tr):
        if a['outcome']!='completed':continue
        xy=np.array(t['xy']);tt=np.array(t['t']);ax.plot(xy[:,0],xy[:,1],c='#777',lw=.8)
        k=np.searchsorted(tt,a['end']);ax.plot(*xy[min(k,len(xy)-1)],'o',c='orange',ms=5)
        ax.plot(float(a['stop_x']),float(a['stop_y']),'x',c='crimson',ms=8);d.append(float(a['stop_goal_distance_m']))
    ax.plot([],[],'o',c='orange',label='완료 판정 위치');ax.plot([],[],'x',c='crimson',label='최종 정지 위치')
    ax.set_xlim(*xl);ax.set_ylim(*yl);ax.set_aspect('equal');ax.grid(alpha=.25);ax.legend(fontsize=9,loc='lower right')
    ax.set_title(f'시나리오{n}: 정지 위치가 목표에서 {min(d):.0f}~{max(d):.0f} m',fontsize=11)
fig.suptitle('사례 20  completed 판정 후 능동 제동이 없어 30 m 넘게 굴러감',fontsize=13,y=1.02)
note(fig,'가상 brake가 MAVLink 적극 제동으로 전달되지 않음 · 막다른 끝 근처 목표에서는 지도 밖까지 이동 (다음 경로에서 위치 재정렬)')
fig.savefig('case20_coasting.png',dpi=90,bbox_inches='tight');plt.close(fig)
print('D done')
