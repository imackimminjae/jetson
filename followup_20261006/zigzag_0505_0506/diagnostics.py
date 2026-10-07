from pathlib import Path
import pickle,json,sys
from datetime import datetime
from zoneinfo import ZoneInfo
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager,dates as mdates
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[1];KST=ZoneInfo('Asia/Seoul')
sys.path.insert(0,str(ROOT/'followup_20261006/scripts'))
from analyze_runs import eff_steer,project
S=pickle.load((HERE/'series.pkl').open('rb'));P=json.loads((HERE/'plan_events.json').read_text());U=json.loads((HERE/'summary.json').read_text())
font_manager.fontManager.addfont('/usr/share/fonts/truetype/nanum/NanumGothic.ttf')
plt.rcParams.update({'font.family':'NanumGothic','axes.unicode_minus':False})
clock=lambda t:datetime.fromtimestamp(float(t),KST).strftime('%H:%M:%S.%f')[:-3]
dates=lambda ts:[datetime.fromtimestamp(float(t),KST) for t in ts]
out={}
def plant_model(times,cl,tau):
    # Integrate held commands at their original event times, then sample at telemetry times.
    points=np.unique(np.r_[times,cl[:,0]]);j=np.maximum(0,np.searchsorted(cl[:,0],points,side='right')-1);u=cl[j,2]
    y=np.zeros(len(points))
    for i in range(1,len(points)):y[i]=u[i-1]+(y[i-1]-u[i-1])*np.exp(-(points[i]-points[i-1])/tau)
    return np.interp(times,points,y)
for tag in ['050536','050627']:
    d=S[tag];hs=d['hs'];cl=d['cl'];tr=d['lower'];eff=eff_steer(hs[:,2],hs[:,1]);m=(hs[:,0]>d['on']+4)&(hs[:,0]<d['off'])&(hs[:,1]>1)
    fit=[]
    for tau in np.arange(.4,1.101,.01):
        pred=plant_model(hs[:,0],cl,tau);gain=float(pred[m]@eff[m]/max(1e-12,pred[m]@pred[m]));err=float(np.degrees(np.sqrt(np.mean((gain*pred[m]-eff[m])**2))));fit.append((err,tau,gain))
    best=min(fit); mt=(tr[:,0]>d['on']+4)&(tr[:,0]<d['off'])&(tr[:,10]==1)
    truth=np.interp(tr[:,0],hs[:,0],eff)
    model_rmse=float(np.degrees(np.sqrt(np.mean((tr[mt,19]-truth[mt])**2))))
    rows=[]
    for e,z in zip(d['events'],P[tag]):
        row={'time':clock(e['t_start']),'t':e['t_start'],'progress':z['progress'],'valid':e['valid_plan'],'reason':e['reason'],'status':e['status'],'wp':[e['wp0'],e['wp1']],'k1_candidates':e['candidate_counts'][1] if len(e['candidate_counts'])>1 else 0,'k1_shift_previous':z['previous_plan_lateral_m'][0] if z['previous_plan_lateral_m'] else None}
        if e['valid_plan']:
            widths=[];gaps=[];ref=np.array(e['preview_world']);plan=np.array(e['planned_world'])
            for c in e['intervals_world']:
                if c[0]!=1:continue
                a,b=np.array(c[2:4]),np.array(c[4:6]);length=np.linalg.norm(b-a);u=(ref[1]-a)@((b-a)/length);ins=(c[6]-c[7])/2
                gaps.append((float(max(0.,-u,u-length)),float(max(0.,-u-ins,u-length-ins))))
                widths.append([c[6],c[7]])
            row.update(soft_gap_min=min(x[0] for x in gaps),raw_gap_min=min(x[1] for x in gaps),widths=widths,plan_headings=z['plan_heading_deg'],k1_new_vs_reference_m=float(np.linalg.norm(plan[1]-ref[1])))
        rows.append(row)
    pg=np.interp(tr[:,0],d['od_t'],d['prog']); evinvalid=[r for r in rows if not r['valid']]
    neutral=tr[(tr[:,12]==3)&(tr[:,0]>=d['on'])&(tr[:,0]<=d['off'])]
    out[tag]={'plant_fit_rmse_deg':best[0],'plant_tau_sec':float(best[1]),'plant_gain':best[2], 'internal_model_rmse_deg':model_rmse,'neutral_samples':len(neutral),'trace_max_gap':float(np.diff(tr[:,0]).max()),'upper_invalid':evinvalid,'events':rows}
    d['eff']=eff
    print(tag,'plant tau/gain/rmse',best[1],best[2],best[0],'internalrmse',model_rmse,'invalid',[(r['time'],r['reason']) for r in evinvalid],'neutral',len(neutral))
(HERE/'diagnostics.json').write_text(json.dumps(out,indent=2)+'\n')

GP=np.array([[150,-140],[130,-103],[108,-59],[110,-29],[118,-15],[173,-2]],float)
fig,axes=plt.subplots(1,3,figsize=(15,6),layout='constrained')
for tag,col in [('013717','#888'),('034428','#aaa'),('050536','#d38d31'),('050627','#1e83b1')]:
    d=S[tag]; lab=tag[:2]+':'+tag[2:4]+':'+tag[4:]+f" ({U[tag]['rate_hz']:g} Hz)"
    axes[0].plot(d['oxy'][:,0],d['oxy'][:,1],c=col,label=lab)
    if tag.startswith('05'):
        axes[1].plot(d['p'],d['cmd'],c=col,label=lab)
        tr=d['lower'];m=tr[:,10]==1;axes[2].plot(d['lower_progress'][m],tr[m,6],c=col)
axes[0].plot(*GP.T,'k--',lw=1,label='지정 경로');axes[0].set_aspect('equal');axes[0].set_xlim(97,176);axes[0].set_ylim(-66,28);axes[0].legend(fontsize=8)
axes[0].set_title('첫 S자부터 두 번째 분기까지');axes[0].set_xlabel('지도 X (m)');axes[0].set_ylabel('지도 Y (m)')
axes[1].set_title('최신 두 주행의 조향 명령');axes[1].set_ylabel('조향 (°)');axes[1].legend(fontsize=9)
axes[2].set_title('지역 참조 경로 대비 횡오차');axes[2].set_ylabel('횡오차 (m)')
for ax in axes[1:]:
    ax.set_xlim(80,190);ax.set_xlabel('지정 경로에 투영한 진행 거리 (m)');ax.grid(alpha=.2)
    for p in [88,125,150]:ax.axvline(p,c='#999',ls=':',lw=.7)
fig.suptitle('최신 scenario1: 05:05는 북쪽 오분기, 05:06은 동쪽 정분기 후 지그재그\n오분기 이후는 같은 도로 구간이 아니므로 직접 성능 비교에서 제외',fontsize=14)
fig.savefig(HERE/'latest_tracks.png',dpi=170);plt.close(fig)

d=S['050627'];tr=d['lower'];hs=d['hs'];cl=d['cl'];events=[r for r in out['050627']['events'] if r['valid']]
t0=next(r['t'] for r in events if r['progress']>139);t1=d['off'];ev=[r for r in events if t0<=r['t']<=t1]
fig,axes=plt.subplots(4,1,figsize=(13,11),sharex=True,layout='constrained')
et=np.array([r['t'] for r in ev]);heads=np.array([r['plan_headings'][0] for r in ev]);m=(tr[:,0]>=t0)&(tr[:,0]<=t1)
axes[0].step(dates(et),heads,where='post',c='#cc4d49',label='상위 계획 첫 구간 방향')
axes[0].plot(dates(tr[m,0]),np.degrees(tr[m,3]),c='#2d81b4',label='차량 방향')
axes[0].set_ylabel('지도 방향 (°)');axes[0].legend(loc='upper right',fontsize=9)
mc=(cl[:,0]>=t0)&(cl[:,0]<=t1);mh=(hs[:,0]>=t0)&(hs[:,0]<=t1)
axes[1].plot(dates(cl[mc,0]),np.degrees(cl[mc,2]),c='#cc4d49',label='전송 조향 명령 (ROS 부호)')
axes[1].plot(dates(hs[mh,0]),np.degrees(d['eff'][mh]),c='#2d81b4',label='SIH 회전율/속도에서 환산한 실제 조향')
axes[1].plot(dates(tr[m,0]),np.degrees(tr[m,19]),c='#555',ls='--',lw=1,label='하위 내부 조향 추정')
axes[1].set_ylabel('조향 (°)');axes[1].legend(loc='upper right',fontsize=8)
axes[2].plot(dates(et),[r['k1_shift_previous'] if r['k1_shift_previous'] is not None else np.nan for r in ev],'o-',ms=3,c='#965cad',label='직전 계획 대비 k=1 횡이동')
axes[2].bar(dates(et),[r['soft_gap_min'] for r in ev],width=.22/86400,alpha=.4,color='#dc9a30',label='참조의 허용단면 이탈 거리(절댓값)')
axes[2].set_ylabel('거리 (m)');axes[2].legend(loc='upper right',fontsize=9)
axes[3].plot(dates(tr[m,0]),tr[m,6],c='#965cad',label='지역 참조 경로 대비 횡오차')
axes[3].set_ylabel('횡오차 (m)');axes[3].legend(loc='upper right',fontsize=9)
for ax in axes:
    ax.axhline(0,c='#999',lw=.7);ax.grid(alpha=.15)
    for t in et:ax.axvline(datetime.fromtimestamp(t,KST),c='#bbb',lw=.5,alpha=.35)
axes[-1].xaxis.set_major_formatter(mdates.DateFormatter('%H:%M:%S',tz=KST));axes[-1].set_xlabel('2026-10-06 KST · 가는 세로선: 유효 상위 계획 갱신')
fig.suptitle('05:06 주행 우회전 뒤: 계획 변경 - 조향 반전 - 늦은 차량 반응\n후보가 1개인 구간에서도 반복 · 실제 조향은 SIH 속도와 yaw rate의 자전거 모델 환산값',fontsize=14)
fig.savefig(HERE/'zigzag_timeline.png',dpi=170);plt.close(fig)
