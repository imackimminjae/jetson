import sys;sys.path.insert(0,'/home/imac/ros2_ws/followup_20261009/edge_cases');sys.path.insert(0,'/home/imac/ros2_ws/followup_20261009/route_design')
from common import *
import csv,math,os
RD='/home/imac/ros2_ws/followup_20261009/route_design/';os.chdir(RD)
M=np.load(RD+'road_mask.npy');ext=json.load(open(RD+'road_mask.json'))['ext']
def mbg(ax): ax.imshow(np.where(M,1.0,0.35),origin='lower',extent=(ext[0],ext[1],ext[2],ext[3]),cmap='gray',vmin=0,vmax=1)
KR='/home/imac/ros2_ws/src/virtual_control/data/knu_routes/'
# ---- case 17: dense waypoints fall behind terminal anchor (schematic on old scenario6)
P6=np.array([[float(r['x']),float(r['y'])] for r in csv.DictReader(open('/home/imac/ros2_ws/followup_20261009/edge_cases/old_scenario6.csv'))]);P3=np.array([[float(r['x']),float(r['y'])] for r in csv.DictReader(open(KR+'scenario3.csv'))])
fig,axs=plt.subplots(1,2,figsize=(14,6.5))
for ax,(P,title) in zip(axs,[(P6,'기존 scenario6: waypoint 간격 5 m'),(P3,'scenario3: waypoint 간격 15~28 m')]):
    i0=20 if P is P6 else 1;veh=P[i0-1]+0.4*(P[i0]-P[i0-1]);d=(P[i0]-P[i0-1])/np.linalg.norm(P[i0]-P[i0-1])
    # find waypoint pair as planner would: wp0 = first waypoint more than ~0 m ahead within switch distance 10 m logic -> approximate
    ahead=[k for k in range(len(P)) if (P[k]-veh)@d>0];wp0=ahead[0]
    while wp0<len(P)-2 and np.linalg.norm(P[wp0]-veh)<=10: wp0+=1
    wp1=wp0+1;anchor=veh+d*22.5;term=veh+d*27
    ax.plot(*P.T,'o-',c='#888',ms=4,lw=1,label='waypoint');ax.plot(*veh,'k^',ms=10,label='차량')
    ax.plot([veh[0],term[0]],[veh[1],term[1]],'-',c='teal',lw=6,alpha=.35,label='상위 계획 지평 (6단계×4.5 m≈27 m)')
    ax.plot(*anchor,'s',c='teal',ms=9,label='끝점 기준점 (k=5)');tw=P[wp0] if (P[wp0]-anchor)@d>4.5 else P[wp1];ax.plot(*tw,'*',c='crimson',ms=16,label=f'끝점 목표에 쓰이는 waypoint (차량에서 {np.linalg.norm(tw-veh):.0f} m)')
    ax.annotate('',xy=tw,xytext=anchor,arrowprops=dict(arrowstyle='->',color='crimson',lw=2))
    far=max(27,float(np.linalg.norm(tw-veh)));c=veh+d*far/2;hw=far/2+10;ax.set_xlim(c[0]-hw,c[0]+hw);ax.set_ylim(c[1]-hw,c[1]+hw);ax.set_aspect('equal');ax.grid(alpha=.25);ax.set_title(title,fontsize=11);ax.legend(fontsize=8,loc='lower left')
fig.suptitle('사례 17  waypoint가 촘촘하면 끝점 목표 방향(기준점→wp1)이 뒤를 향함',fontsize=13,y=1.02)
note(fig,'개념도: 플래너는 waypoint가 10 m 안에 들어오면 다음으로 넘어가고, 끝점 목표 방향을 k=5 기준점에서 wp1 쪽으로 잡음(최대 20° 꺾임)')
fig.savefig(BC+'../edge_cases/case17_dense_waypoints.png',dpi=90,bbox_inches='tight');plt.close(fig)
print('C done')
