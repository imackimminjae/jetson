from pathlib import Path
import json,csv
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager

p=Path(__file__).resolve().parent
experiments=json.loads((p/'experiments.json').read_text())
runs=[json.loads(x) for x in (p/'results.jsonl').read_text().splitlines()]
wrap=lambda a:np.arctan2(np.sin(a),np.cos(a))
def arc(q):return np.r_[0,np.cumsum(np.linalg.norm(np.diff(q,axis=0),axis=1))]
def project(q,point):
    delta=np.diff(q,axis=0);lengths=np.linalg.norm(delta,axis=1)
    u=np.clip(np.sum((point-q[:-1])*delta,axis=1)/np.maximum(lengths**2,1e-15),0,1)
    foot=q[:-1]+u[:,None]*delta;k=np.argmin(np.linalg.norm(foot-point,axis=1))
    return arc(q)[k]+u[k]*lengths[k]
def sample(q,s):
    a=arc(q);k=np.clip(np.searchsorted(a,s,side='right')-1,0,len(q)-2)
    v=q[k+1]-q[k];return q[k]+v*(s-a[k])/max(a[k+1]-a[k],1e-12),np.arctan2(v[1],v[0])
def measure(r,progress_limit=None):
    rows=r['states']
    if progress_limit is not None:
        hit=next((i for i,z in enumerate(rows) if z['progress']>=progress_limit),len(rows)-1)
        rows=rows[:hit+1]
    if not rows:return {}
    end=rows[-1]['t'] if progress_limit is not None else float('inf');moving=[z for z in rows if z['v']>2]
    cmd=np.array([z['command'] for z in moving]);times=np.array([z['t'] for z in moving])
    distance=np.array([z['route_distance'] for z in moving]);valid=[z for z in r['plans'] if z['t']<=end and z['valid']]
    plans=[z for z in r['plans'] if z['t']<=end];lateral=[];heading=[]
    for a,b in zip(valid,valid[1:]):
        if b['t']-a['t']>.201:continue
        old,new=np.array(a['path']),np.array(b['path']);s=project(old,new[0])
        if s+5>arc(old)[-1] or 5>arc(new)[-1]:continue
        q,h=sample(old,s+5);v,g=sample(new,5)
        lateral.append(float((v-q)@[-np.sin(h),np.cos(h)]));heading.append(float(np.degrees(wrap(g-h))))
    return dict(outcome=r['outcome'],duration=rows[-1]['t'],progress_m=max(z['progress'] for z in rows),
        upper_attempts=len(plans),upper_failures=sum(not z['valid'] for z in plans),
        lower_failures=sum(not z['lower_valid'] for z in rows),
        center_outside_samples=sum(z['road_center_margin']<0 for z in rows),
        minimum_center_margin_m=min(z['road_center_margin'] for z in rows),
        route_distance_rms_m=float(np.sqrt(np.mean(distance**2))) if len(distance) else 0,
        route_distance_max_m=float(max(distance)) if len(distance) else 0,
        steering_tv_deg_per_sec=float(np.degrees(abs(np.diff(cmd))).sum()/max(times[-1]-times[0],.1)) if len(cmd)>1 else 0,
        saturated_percent=float(100*np.mean(abs(cmd)>.299)) if len(cmd) else 0,
        plan_shift_p95_m=float(np.quantile(abs(np.array(lateral)),.95)) if lateral else 0,
        plan_heading_jump_p95_deg=float(np.quantile(abs(np.array(heading)),.95)) if heading else 0)

summary=[];bykey={(r['scene'],r['plant']['name'],r['config']['name']):r for r in runs}
for r in runs:
    key=(r['scene'],r['plant']['name'],r['config']['name']);base=bykey[key[:2]+('baseline',)]
    common=min(max(z['progress'] for z in r['states']),max(z['progress'] for z in base['states']))
    summary.append(dict(scene=key[0],plant=key[1],config=key[2],**measure(r),common_progress_m=common,
                        paired_candidate=measure(r,common),paired_baseline=measure(base,common)))
(p/'metrics.json').write_text(json.dumps(summary,indent=2,ensure_ascii=False))
fields=[k for k in summary[0] if not k.startswith('paired_')]
with (p/'metrics.csv').open('w') as f:
    writer=csv.DictWriter(f,fields,extrasaction='ignore');writer.writeheader();writer.writerows(summary)
for name in [c['name'] for c in experiments['configs']]:
    rows=[r for r in summary if r['config']==name]
    print(name,'goals',sum(r['outcome']=='goal_reached' for r in rows),'/',len(rows),
          'offroad_runs',sum(r['center_outside_samples']>0 for r in rows),
          'failures',sum(r['upper_failures'] for r in rows),
          'curve_TV',round(np.mean([r['steering_tv_deg_per_sec'] for r in rows if r['scene']!='straight']),2))

font_manager.fontManager.addfont('/usr/share/fonts/truetype/nanum/NanumGothic.ttf')
plt.rcParams.update({'font.family':'NanumGothic','axes.unicode_minus':False,'font.size':10})
selected=['baseline','turn03','turn015','dpsi2','terminal100','wp6']
colors=['#222222','#127bbb','#27a775','#a258b0','#e1962e','#e55858']
fig,axes=plt.subplots(2,3,figsize=(17,10),layout='constrained')
for j,scene in enumerate(experiments['scenes']):
    ax=axes[0,j];center=np.array(scene['center']);ax.plot(center[:,0],center[:,1],color='#cccccc',lw=12,label='도로 중심선 (폭 표시 아님)')
    if scene['spur']:
        q=np.array(scene['spur']);ax.plot(q[:,0],q[:,1],color='#ccc',lw=12)
    route=np.array(scene['route']);ax.plot(route[:,0],route[:,1],'o',ms=3,color='#999')
    for name,color in zip(selected,colors):
        r=bykey[(scene['name'],'nominal',name)];a=r['states'];ax.plot([z['x'] for z in a],[z['y'] for z in a],c=color,label=name,lw=1.5)
    ax.set_aspect('equal');ax.set_title({'straight':'직선','curve':'곡선','branch_width':'분기 + 도로폭 변화'}[scene['name']]);ax.set_xlabel('world x (m)');ax.set_ylabel('world y (m)');ax.grid(alpha=.2)
    if j==0:ax.legend(fontsize=8)
    ax=axes[1,j]
    for name,color in zip(selected,colors):
        r=bykey[(scene['name'],'nominal',name)];a=r['states'];ax.plot([z['t'] for z in a],np.degrees([z['command'] for z in a]),c=color,label=name,alpha=.85,lw=1)
    ax.set_xlabel('가상 시간 (s)');ax.set_ylabel('조향 명령 (deg)');ax.grid(alpha=.2)
fig.suptitle('실제 상·하위 제어 코드 + 가상 자전거 차량: 동일 조건의 폐루프 비교',fontsize=16)
fig.supxlabel('인공 도로 / 이상적 점유 지도 / 속도는 1.5 m/s²로 6 m/s까지 증가. PX4·Isaac 실주행 결과가 아님.',fontsize=10)
fig.savefig(p/'closed_loop_comparison.png',dpi=150);plt.close(fig)

fig,axs=plt.subplots(2,2,figsize=(14,10),layout='constrained')
ax=axs[0,0]
for name,color,label in [('baseline','#333','현재 설정'),('dpsi2','#197db9','upper_r_dpsi = 2')]:
    r=bykey[('branch_width','perturbed',name)];a=r['states'];ax.plot([z['t'] for z in a],np.degrees([z['command'] for z in a]),c=color,label=label)
ax.set_xlim(20,34);ax.set_xlabel('가상 시간 (s)');ax.set_ylabel('조향 명령 (deg)');ax.set_title('분기·폭 변화 / 응답 지연·yaw 잡음 조건');ax.legend();ax.grid(alpha=.2)
ax=axs[0,1]
names=['dpsi2','terminal100','turn03','turn015','filter09','lower80','wp6','samples4']
base_mean=np.mean([z['steering_tv_deg_per_sec'] for z in summary if z['config']=='baseline' and z['scene']!='straight'])
changes=[100*(np.mean([z['steering_tv_deg_per_sec'] for z in summary if z['config']==name and z['scene']!='straight'])/base_mean-1) for name in names]
bars=ax.barh(names,changes,color=['#197db9' if v<0 else '#d55a57' for v in changes]);ax.bar_label(bars,fmt='%+.1f%%',padding=3);ax.set_xlim(min(changes)-7,max(changes)+7);ax.axvline(0,c='#777',lw=1);ax.invert_yaxis();ax.set_xlabel('현재 대비 조향 총변동/초 변화 (%)');ax.set_title('곡선·분기 4조건 평균 — 음수일수록 변동 감소');ax.grid(axis='x',alpha=.2)
ax=axs[1,0];scene=next(s for s in experiments['scenes'] if s['name']=='branch_width')
bitmap=np.fromfile(scene['bitmap'],dtype=np.int8).reshape(scene['height'],scene['width']);o=scene['origin'];res=scene['resolution']
ax.imshow(bitmap[::3,::3],origin='lower',extent=[o[0],o[0]+scene['width']*res,o[1],o[1]+scene['height']*res],cmap='gray_r',vmin=0,vmax=400,interpolation='nearest')
for name,color,label in [('baseline','#333','현재 설정'),('dpsi2','#197db9','방향 보정 비용 2'),('turn0','#d44848','회전 비용 0 → 계획 만료')]:
    r=bykey[('branch_width','nominal',name)];a=r['states'];ax.plot([z['x'] for z in a],[z['y'] for z in a],c=color,label=label,lw=2);ax.scatter(a[-1]['x'],a[-1]['y'],c=color,s=25)
goal=scene['route'][-1];ax.scatter(*goal,c='#138b5a',marker='*',s=140,label='목표');ax.set_xlim(60,142);ax.set_ylim(65,145);ax.set_aspect('equal');ax.set_xlabel('world x (m)');ax.set_ylabel('world y (m)');ax.set_title('회전 비용 0: 도로 안에서도 잘못된 분기로 진행');ax.legend(fontsize=8)
ax=axs[1,1];ax.axis('off')
table=[]
for name in ['baseline','dpsi2','terminal100','turn03','turn015','turn0','lower80','filter09']:
    a=[z for z in summary if z['config']==name]
    table.append([name,f"{sum(z['outcome']=='goal_reached' for z in a)}/6",sum(z['upper_failures'] for z in a)])
tab=ax.table(cellText=table,colLabels=['설정','목표 도달','상위 실패 합계'],loc='center',cellLoc='center');tab.auto_set_font_size(False);tab.set_fontsize(11);tab.scale(1,1.8)
ax.set_title('조향 변동 감소와 계산 실패는 별도 평가')
fig.suptitle('60회 독립 폐루프 시험: 후보의 효과와 역효과를 함께 확인',fontsize=16)
fig.supxlabel('실제 상·하위 계산 코드 + 인공 도로 + 비선형 자전거 모델. 실측의 심한 진동 전체를 재현한 결과는 아님.',fontsize=10)
fig.savefig(p/'simulation_findings.png',dpi=150);plt.close(fig)
