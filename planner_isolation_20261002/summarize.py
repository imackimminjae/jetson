from pathlib import Path
import json,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.colors import ListedColormap
font_manager.fontManager.addfont('/usr/share/fonts/truetype/nanum/NanumGothic.ttf')
plt.rcParams.update({'font.family':'NanumGothic','axes.unicode_minus':False,'font.size':10})
p=Path(__file__).resolve().parent
load=lambda name:{r['id']:r for r in map(json.loads,(p/name).read_text().splitlines())}
base=load('results.jsonl');factor=load('factor_results.jsonl');feedback=load('feedback_results.jsonl');inputs=json.loads((p/'inputs.json').read_text())
rep=json.loads((p/'reproduction_check.json').read_text());aligned=json.loads((p/'aligned_changes.json').read_text())
summary={'reproduction':{'attempts':len(rep),'outcome_matches':sum(x[1] for x in rep),'max_success_path_error_m':max(x[2] for x in rep if x[2] is not None)},'repeat':[],'feedback':{},'sweeps':{},'factorials':{}}
for i in inputs['selected']:
 trials=base[f'recorded_{i}']['trials'];h=[x['h1_deg'] for x in trials if x['valid']]
 summary['repeat'].append({'snapshot':i,'runs':len(trials),'valid':len(h),'heading_span_deg':max(h)-min(h) if h else None})
for name,r in feedback.items():
 trials=[x['trials'][0] for x in r['series']];h=np.array([x.get('h1_deg',np.nan) for x in trials]);dh=np.diff(h)
 summary['feedback'][name]={'valid_count':sum(x['valid'] for x in trials),'h1_deg':h.tolist(),'branches':[x['branch_step'] for x in trials],'max_step_change_deg':float(max(abs(dh))),'last10_span_deg':float(np.ptp(h[-10:]))}
for name in ['base','turn0','dpsi2','dpsi5']:
 rows=[(i,(base[f'recorded_{i}'] if name=='base' else factor[f'sweep_{name}_{i}'])['trials'][0]) for i in range(153)]
 valid=[(i,r) for i,r in rows if r['valid']];h=np.array([r['h1_deg'] for i,r in valid]);delta=np.degrees(np.arctan2(np.sin(np.radians(np.diff(h))),np.cos(np.radians(np.diff(h)))));good=np.diff([i for i,r in valid])==1
 summary['sweeps'][name]={'valid':len(valid),'failures':153-len(valid),'consecutive_jump_median_p95_max_deg':np.quantile(abs(delta[good]),[.5,.95,1]).tolist(),'consecutive_jumps_over_10_deg':int(sum(abs(delta[good])>10))}
for a,b in [(83,84),(84,85),(92,93),(94,95)]:
 cases=[]
 for key,r in factor.items():
  if not key.startswith(f'factor_{a}_{b}_'):continue
  bits=key.split('_')[-1]
  if bits.count('1') not in [0,1,len(bits)]:continue
  t=r['trials'][0];cases.append({'bits_pose_grid_previous_B_wp':bits,'valid':t['valid'],'h1_deg':t.get('h1_deg'),'branch_step':t['branch_step'],'B_after':r['B_after']})
 summary['factorials'][f'{a}_{b}']=cases
summary['aligned_changes_largest']=sorted([x for x in aligned if '5'in x],key=lambda x:abs(x['5']['lateral_m']),reverse=True)[:5]
(p/'summary.json').write_text(json.dumps(summary,indent=2,ensure_ascii=False))

fig,axs=plt.subplots(2,2,figsize=(15,10),layout='constrained')
ax=axs[0,0]
for key,color,label in [('feedback_85_0.6','#147dc2','현재 회전 비용 0.6'),('feedback_85_0.0','#999','진단용 회전 비용 0')]:
 x=summary['feedback'][key];h=np.array(x['h1_deg']);ax.plot(np.arange(1,41),h,'-o',ms=3,c=color,label=label)
 if key.endswith('0.6'):
  mask=np.array(x['branches'])>=1;ax.scatter(np.arange(1,41)[mask],h[mask],c='#e0446d',s=26,zorder=4,label='분기 감지 → 회전 비용 활성')
ax.set_title('하위 제어기 없이 상위에서 방향 진동 재현');ax.set_xlabel('고정 장면의 반복 계획 횟수');ax.set_ylabel('첫 계획 구간 방향 (지도 좌표, deg)');ax.grid(alpha=.2);ax.legend(fontsize=9)
ax=axs[0,1];i=85;s=inputs['snapshots'][i];e=s['event'];yaw=e['yaw_rad'];R=np.array([[np.cos(yaw),-np.sin(yaw)],[np.sin(yaw),np.cos(yaw)]]);xy=np.array([e['x'],e['y']]);matches=json.loads((p/'grid_matches.json').read_text());g=inputs['grids'][str(matches[str(i)][1])];grid=np.array(g['data']).reshape(g['height'],g['width']);extent=[g['origin'][0],g['origin'][0]+g['width']*g['res'],g['origin'][1],g['origin'][1]+g['height']*g['res']];ax.imshow(np.where(grid==0,1,0),origin='lower',extent=extent,cmap=ListedColormap(['#65707f','#faf9ef']),interpolation='nearest')
for result,color,label in [(base[f'recorded_{i}'],'#1779cd','원래 계획'),(factor[f'prune_{i}'],'#e98522','미선택 20cm 후보만 제외')]:
 pts=(np.array(result['trials'][0]['world'])-xy)@R;ax.plot(pts[:,0],pts[:,1],'-o',c=color,lw=2,ms=3,label=label)
tiny=next(v for v in e['intervals_world'] if int(v[0])==5 and int(v[1])==1);pt=(np.mean(np.array(tiny[2:6]).reshape(2,2),axis=0)-xy)@R;ax.scatter(*pt,s=100,facecolors='none',edgecolors='#e53561',linewidths=2);ax.annotate('선택되지 않은 20cm 후보\n이 후보만으로 회전 비용이 켜짐',xy=pt,xytext=(3,18),arrowprops={'arrowstyle':'->','color':'#e53561'},fontsize=9)
ax.scatter(0,0,c='black',marker='>',s=65);ax.set_xlim(-1,43);ax.set_ylim(-20,22);ax.set_aspect('equal');ax.set_xlabel('차량 전방 x (m)');ax.set_ylabel('차량 좌측 y (m)');ax.set_title('동일 장면: 후보 하나로 첫 방향 15.3° 차이');ax.legend(fontsize=8,loc='lower left')
ax=axs[1,0];names=['원래 비용','짧은 미선택\n후보 제외','회전 비용 0\n(진단용)','방향 보정 비용\n1 → 2'];ref=base['recorded_83']['trials'][0]['h1_deg'];values=[base['recorded_84']['trials'][0]['h1_deg']-ref,factor['prune_84']['trials'][0]['h1_deg']-ref,base['turn_84_0.0']['trials'][0]['h1_deg']-ref,base['dpsi_84_2.0']['trials'][0]['h1_deg']-ref];bars=ax.bar(names,values,color=['#e65858','#e7a13a','#777','#1683bb']);ax.bar_label(bars,fmt='%.2f°');ax.set_ylabel('기록된 직전 계획 대비 방향 변화 (deg)');ax.set_title('18:45:39.874 — 다른 입력은 그대로인 한 번의 재계산');ax.set_ylim(0,23);ax.grid(axis='y',alpha=.2)
ax=axs[1,1];q=[x for x in aligned if '5'in x];t0=inputs['snapshots'][0]['event']['t_start'];ax.plot([x['t']-t0 for x in q],[x['5']['lateral_m'] for x in q],'-o',ms=3,c='#176fc0');ax.axhline(0,c='#777',lw=1);ax.set_title('차량 진행량을 맞춰도 앞 5m 경로가 좌우로 이동');ax.set_xlabel('로그 시작 후 경과 시간 (s)');ax.set_ylabel('이전 경로 대비 횡방향 변화 (m)');ax.grid(alpha=.2)
fig.suptitle('상위 플래너 원인 분리: 재현 검증 + 입력 고정 실험',fontsize=16)
fig.supxlabel('좌상: 차량·지도·경유점 고정, 이전 계획과 폭 상태만 갱신. 나머지: 기록 입력의 오프라인 재계산. 실제 주행 개선율을 뜻하지 않음.',fontsize=10)
fig.savefig(p/'planner_isolation.png',dpi=150);plt.close(fig)
print(json.dumps({k:summary[k] for k in ['reproduction','sweeps']},indent=2))
for k,v in summary['feedback'].items():print(k,'max_jump',v['max_step_change_deg'],'last10_span',v['last10_span_deg'])
