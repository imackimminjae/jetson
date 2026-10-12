import sys;sys.path.insert(0,'/home/imac/ros2_ws/followup_20261009/edge_cases');sys.path.insert(0,'/home/imac/ros2_ws/followup_20261009/route_design')
from common import *
import csv,math,os
RD='/home/imac/ros2_ws/followup_20261009/route_design/';os.chdir(RD)
M=np.load(RD+'road_mask.npy');ext=json.load(open(RD+'road_mask.json'))['ext']
def mbg(ax): ax.imshow(np.where(M,1.0,0.35),origin='lower',extent=(ext[0],ext[1],ext[2],ext[3]),cmap='gray',vmin=0,vmax=1)
KR='/home/imac/ros2_ws/src/virtual_control/data/knu_routes/'
# ---- case 11+12: scenario2 route leaves the map; route through island
H2='/home/imac/ros2_ws/followup_20261008/batches_182344_183952_185426_review/scenario2/'
tr=json.load(gzip.open(H2+'tracks.json.gz','rt'));S2=json.load(open(H2+'summary.json'));rt=np.array(S2['route'])
fig,axs=plt.subplots(1,2,figsize=(17,7),gridspec_kw=dict(width_ratios=[1.6,1]))
for ax,(xl,yl) in zip(axs,[((-260,5),(-5,112)),((-140,-100),(40,75))]):
    bg(ax,'regress/s2/');ax.contour(np.arange(M.shape[1])*.25+ext[0],np.arange(M.shape[0])*.25+ext[2],M.astype(float),levels=[.5],colors='royalblue',linewidths=.8)
    ax.plot(rt[:,0],rt[:,1],'k--o',lw=1.5,ms=5,label='지정 경로 (scenario2.csv)')
    for t in tr:
        xy=np.array(t['xy']);ax.plot(xy[:,0],xy[:,1],lw=1,c='crimson' if t['attempt'] in (7,10) else 'teal',alpha=.8)
    ax.set_xlim(*xl);ax.set_ylim(*yl);ax.set_aspect('equal');ax.grid(alpha=.25)
axs[0].axvspan(-66,5,color='orange',alpha=.12);axs[0].text(-60,30,'x>-66: BEV 전부 100(주행불가)\n처리된 도로 데이터에도 도로 없음\n→ 8/10회 (-73,70)에서 계획 소실',fontsize=10,color='#a50')
axs[0].set_title('사례 11  경로 끝 70 m가 지도 밖 (파란 선=처리된 도로 데이터 경계, 배경=누적 BEV)',fontsize=11);axs[0].legend(loc='lower left',fontsize=9)
axs[1].set_title('사례 12  경로 선이 섬 한가운데를 지남\n빨강=남쪽으로 빠진 7·10회',fontsize=11)
note(fig,'시나리오2 20261008_183952 10회 · 첫 waypoint(-250,101)도 도로 밖이라 MATLAB이 시작 위치를 (-252.8,93.5)로 덮어씀')
fig.savefig(BC+'../edge_cases/case11_12_s2_map_and_island.png',dpi=90,bbox_inches='tight');plt.close(fig)
# ---- case 15: acute junctions infeasible within steering limit
from gen import H
from band import optimize,radius
spots=[((0,0,1),(1,2,1),'n1 교차로 104° (1번→2번)',(-218,79)),((40,36,30),(41,30,35),'북동 n30 116° (30번→31번)',(203,160)),((49,34,36),(39,36,29),'북동 n36 98° (37번→30번)',(188,178))]
HB={(h['e'],h['a'],h['b']):h for h in H}
def tail(P,L):
    s=np.concatenate([[0],np.cumsum(np.linalg.norm(np.diff(P,axis=0),axis=1))]);return P[s>=s[-1]-L]
def head(P,L):
    s=np.concatenate([[0],np.cumsum(np.linalg.norm(np.diff(P,axis=0),axis=1))]);return P[s<=L]
fig,axs=plt.subplots(1,3,figsize=(18,6.5))
for ax,(ki,ko,title,c) in zip(axs,spots):
    hi,ho=HB[ki],HB[ko];P=np.vstack([tail(hi['P'],25),head(ho['P'],25)]);Z,rm,cl=optimize(P,clear=1.1,mu=2000)
    mbg(ax);ax.plot(*P.T,'b--',lw=1,label='도로 중심선');ax.plot(*Z.T,'r-',lw=2,label=f'최적 경로 (최소 반경 {rm:.1f} m)')
    rad=radius(Z);b=Z[10:-10];m=rad<9.1;ax.plot(b[m,0],b[m,1],'m.',ms=5,label='반경<9.1 m 구간')
    ax.set_xlim(c[0]-22,c[0]+22);ax.set_ylim(c[1]-22,c[1]+22);ax.set_aspect('equal');ax.grid(alpha=.25);ax.set_title(title,fontsize=11);ax.legend(fontsize=8,loc='lower left')
fig.suptitle('사례 15  모서리 곡선 처리가 없는 예각 교차로: 도로 경계 여유 1.1 m를 지키면 최대 조향 반경(약 9.1 m)으로 못 돎',fontsize=13,y=1.03)
note(fig,'배경=processed_road_map.mat 재현 도로 · 최적 경로=곡률 최소화(도로 경계 여유 제약) · 신규 경로 설계에서 이 회전들은 제외')
fig.savefig(BC+'../edge_cases/case15_acute_junctions.png',dpi=90,bbox_inches='tight');plt.close(fig)
# ---- case 16: roundabout ring radius vs min turning radius
import pickle as pk
rd=pk.load(open(RD+'roads.pkl','rb'));ring=np.vstack([r['raw'][:,:2] for r in rd if r['id'] in (13,16,17,18) or r['id']==11])
A=np.c_[2*ring,np.ones(len(ring))];b=(ring**2).sum(1);cx,cy,c0=np.linalg.lstsq(A,b,rcond=None)[0];Rr=math.sqrt(c0+cx**2+cy**2)
fig,ax=plt.subplots(figsize=(8,8));mbg(ax)
for r in rd:
    if r['id'] in (11,13,16,17,18,10,8,9): ax.plot(*r['raw'][:,:2].T,'b-',lw=1)
th=np.linspace(0,2*np.pi,200);ax.plot(cx+Rr*np.cos(th),cy+Rr*np.sin(th),'g--',lw=1.5,label=f'원형 도로 중심선 맞춤 원 R={Rr:.1f} m')
ax.plot(cx+9.1*np.cos(th),cy+9.1*np.sin(th),'r-',lw=1.5,label='최대 조향 시 최소 회전 원 R≈9.1 m')
ax.set_xlim(cx-25,cx+25);ax.set_ylim(cy-25,cy+25);ax.set_aspect('equal');ax.grid(alpha=.25);ax.legend(fontsize=9,loc='lower left')
ax.set_title('사례 16  서쪽 회전교차로: 원형 도로 반경이 조향 한계에 거의 붙어 있음',fontsize=11)
note(fig,'축거 2.8 m, 최대 조향 0.3 rad → R=L/tan(δ)≈9.1 m · 원형 구간을 도는 동안 조향이 거의 포화 상태')
fig.savefig(BC+'../edge_cases/case16_roundabout_radius.png',dpi=90,bbox_inches='tight');plt.close(fig)
# ---- case 17: dense waypoints fall behind terminal anchor (schematic on old scenario6)
P6=np.array([[float(r['x']),float(r['y'])] for r in csv.DictReader(open(KR+'scenario6.csv'))]);P3=np.array([[float(r['x']),float(r['y'])] for r in csv.DictReader(open(KR+'scenario3.csv'))])
fig,axs=plt.subplots(1,2,figsize=(14,6.5))
for ax,(P,title) in zip(axs,[(P6,'기존 scenario6: waypoint 간격 5 m'),(P3,'scenario3: waypoint 간격 15~28 m')]):
    i0=8 if P is P6 else 1;veh=P[i0-1]+0.4*(P[i0]-P[i0-1]);d=(P[i0]-P[i0-1])/np.linalg.norm(P[i0]-P[i0-1])
    # find waypoint pair as planner would: wp0 = first waypoint more than ~0 m ahead within switch distance 10 m logic -> approximate
    ahead=[k for k in range(len(P)) if (P[k]-veh)@d>0];wp0=ahead[0]
    while wp0<len(P)-2 and np.linalg.norm(P[wp0]-veh)<=10: wp0+=1
    wp1=wp0+1;anchor=veh+d*22.5;term=veh+d*27
    ax.plot(*P.T,'o-',c='#888',ms=4,lw=1,label='waypoint');ax.plot(*veh,'k^',ms=10,label='차량')
    ax.plot([veh[0],term[0]],[veh[1],term[1]],'-',c='teal',lw=6,alpha=.35,label='상위 계획 지평 (6단계×4.5 m≈27 m)')
    ax.plot(*anchor,'s',c='teal',ms=9,label='끝점 기준점 (k=5)');ax.plot(*P[wp1],'*',c='crimson',ms=16,label=f'목표로 쓰이는 wp1 (차량에서 {np.linalg.norm(P[wp1]-veh):.0f} m)')
    ax.annotate('',xy=P[wp1],xytext=anchor,arrowprops=dict(arrowstyle='->',color='crimson',lw=2))
    c=veh+d*12;ax.set_xlim(c[0]-25,c[0]+25);ax.set_ylim(c[1]-25,c[1]+25);ax.set_aspect('equal');ax.grid(alpha=.25);ax.set_title(title,fontsize=11);ax.legend(fontsize=8,loc='lower left')
fig.suptitle('사례 17  waypoint가 촘촘하면 끝점 목표 방향(기준점→wp1)이 뒤를 향함',fontsize=13,y=1.02)
note(fig,'개념도: 플래너는 waypoint가 10 m 안에 들어오면 다음으로 넘어가고, 끝점 목표 방향을 k=5 기준점에서 wp1 쪽으로 잡음(최대 20° 꺾임)')
fig.savefig(BC+'../edge_cases/case17_dense_waypoints.png',dpi=90,bbox_inches='tight');plt.close(fig)
print('C done')
