from pathlib import Path
import json,csv,os
os.environ['MPLCONFIGDIR']='/tmp/latest_three_results'
os.sched_setaffinity(0,{min(os.sched_getaffinity(0))})
import numpy as np
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
font_manager.fontManager.addfont('/usr/share/fonts/truetype/nanum/NanumGothic.ttf');plt.rcParams.update({'font.family':'NanumGothic','axes.unicode_minus':False,'font.size':10})
H=Path(__file__).resolve().parent;summaries=[json.loads((H/f'scenario{i}/summary.json').read_text()) for i in [1,2,3]];stats=json.loads((H/'cross_scenario_summary.json').read_text());flat=[]
common='''집계 및 시각화 기준
- running부터coasting 전환 전까지 상위 실패와 조향 지표 집계. 그래프는 정지 확인까지 표시.
- CSV duration_sec에는 출발 준비 시간이 포함됨. 이 표의 주행시간은 running 구간만 계산.
- completed는 목표 영역 접근 판정이며 목표 정차 성공이 아님.
- off_route는 경로 거리15m 초과3초 지속, stopped_10s는0.5m 이상 위치 이동 없는10초.
- 조향은 lower 출력 명령이며 실제 바퀴각 아님.
- 포화 비율은 mode0,속도>1m/s 표본에서 |출력조향 명령 환산값|>=0.29845rad 비율.
- 횡오차p95는 지역참조에 대한 절대오차. 전역 경로 이탈량/분기 정답률과 다름.
- 상위 실패에는 QP 실패와 후보 없음이 포함. 중립 전환 후에도running이면 집계에 포함됨.
- 신호 그래프의 보라선은 회차 판정,주황선은 최종 중립 전환,노란영역은 위치 수신 공백.
- 위치 공백>0.5초를 궤적으로 연결하거나 속도로 보간하지 않음. odom.z는 scalar yaw로 해석.
- 원인 분석은 사용자 요청에 따라 보류. 이 문서는 관측 결과 보고와 시각화 중심.
'''
root=['최신 시나리오1·2·3 주행 결과 — 2026-10-08 KST','세 시나리오 각각10회,총30회. 모두 배치done. 원인분석은보류.','회복확인30초 수정본은 이번 세 배치에 미적용(실행manifest의runner hash가baseline과동일).','이번30회 pose_timeout/배치abort 판정없음. 원인이나 수정효과를 의미하지 않음.','', '시나리오 | 목표접근 | 경로이탈 | 정체 | 상위실패/계산 | 목표접근 회차 최종목표거리']
for sc,(s,stat) in enumerate(zip(summaries,stats),1):
 p=H/f'scenario{sc}';outcome=stat['outcomes'];r=stat['goal_stop_range'];root.append(f"{sc} | {outcome.get('completed',0)} | {outcome.get('off_route',0)} | {outcome.get('stopped_10s',0)} | {stat['failed_upper']}/{stat['total_upper']} | {f'{r[0]:.2f}~{r[1]:.2f}m' if r else '목표접근 회차 없음'}")
 lines=[f'시나리오{sc} 최신10회 결과 보고 (원인분석 보류)',f"원본: {s['session']}",f"최종상태: {s['session_result']['status']['phase']}",f"판정: {outcome}",'', '회차 | 판정 | 시작(KST) | 주행초 | 상위실패/계산 | 명령포화% | 횡오차p95(m) | 판정시목표거리(m) | 정지목표거리(m)']
 for z in s['attempts']:
  lines.append(f"{z['attempt']:2} | {z['outcome']:12} | {z['start_clock']} | {z['end']-z['start']:.2f} | {z['upper_failed']}/{z['upper_count']} | {z['sat_percent']:.2f} | {z['lateral_abs']['p95']:.3f} | {float(z['goal_distance_m']):.2f} | {float(z['stop_goal_distance_m']):.2f}")
  flat.append(dict(scenario=sc,attempt=z['attempt'],outcome=z['outcome'],start_clock=z['start_clock'],running_seconds=z['end']-z['start'],upper_failed=z['upper_failed'],upper_count=z['upper_count'],steering_command_saturation_percent=z['sat_percent'],local_reference_lateral_abs_p95_m=z['lateral_abs']['p95'],goal_distance_at_outcome_m=float(z['goal_distance_m']),goal_distance_at_stop_m=float(z['stop_goal_distance_m'])))
 lines+=['',f"全회차 상위 실패 {stat['failed_upper']}/{stat['total_upper']}.",f"주행 중 최대 위치 메시지 간격 {stat['max_odom_gap']:.3f}초,지도 {stat['max_grid_gap']:.3f}초.",'10회 모두 첫B_before=0,첫waypoint=[1,2].']
 if sc==1:lines+=['9회차만off_route,나머지1~8·10회completed.','주목할 관측:1회 명령포화14.43%,3회 상위실패4/81·횡오차p95=.625m.','목표접근9회의 최종 정지점은 목표에서31.88~37.28m.']
 elif sc==2:lines+=['3·7·10회off_route,나머지1·2·4·5·6·8·9회stopped_10s.','목표접근completed 없음. 정지 목표거리28.78~110.92m.','3회는 주행 중 중립(mode3)→제어(mode0)→중립(mode3) 전환이 기록됨. 상세시간은summary.json.']
 else:lines+=['10회 모두completed. 상위실패는4·5·7회에각2회,다른회차0회.','7회 출력명령 포화10.93%,4·5회 횡오차p95약.555m.','10회 모두 목표접근했지만 최종 정지점은 목표에서31.63~35.06m.']
 lines+=['',common,'필수 전 회차 시각화',str(p/'all_attempts/all_tracks.png'),str(p/f'all_attempts/scenario{sc}_all_attempts.pdf'),'all_attempts/attempt_01.png ~ attempt_10.png: 회차별 궤적·속도·조향·횡오차·B·상위계획지평/실패','per_attempt.csv,summary.json: 전체 지표','overview.png: 요약그래프','sources_sha256.json: 원본식별','', '운영 코드/설정/주행/ARM 변경 없음. 새 원본 기록을 읽고 분석산출물만 추가.','이 결과를 제거할 때 해당 분석폴더만 제거하면 됨. 운영rollback불필요.']
 (p/'RESULT.txt').write_text('\n'.join(lines).replace('全회차','전체 회차')+'\n')
root+=['',common,'파일: scenario1/2/3 각각 RESULT.txt,per_attempt.csv,all_attempts/all_tracks.png 및10페이지PDF.','all_30_attempts.csv:30회 결과 통합표','results_comparison.png:시나리오별 결과와 회차별 지표 비교','30초 입력복구 수정 미적용이며 이번 결과로 그 수정효과를 평가할 수 없음.','운영코드/설정변경없음. 세 배치 모두 완료기록만 읽음.'];(H/'RESULT.txt').write_text('\n'.join(root)+'\n')
with (H/'all_30_attempts.csv').open('w') as f:w=csv.DictWriter(f,fieldnames=list(flat[0]));w.writeheader();w.writerows(flat)
fig,axs=plt.subplots(2,2,figsize=(15,10),layout='constrained');xx=np.arange(1,4);bottom=np.zeros(3)
for label,key,col in [('목표 접근','completed','#178678'),('경로 이탈','off_route','#c84e43'),('10초 정체','stopped_10s','#d5a02e')]:
 vv=np.array([a['outcomes'].get(key,0) for a in stats]);axs[0,0].bar(xx,vv,bottom=bottom,color=col,label=label)
 for x,v,b in zip(xx,vv,bottom):
  if v:axs[0,0].text(x,b+v/2,str(v),ha='center',va='center',color='white',fontsize=13)
 bottom+=vv
axs[0,0].set_xticks(xx,['시나리오1','시나리오2','시나리오3']);axs[0,0].set_ylim(0,11);axs[0,0].set_title('각10회 종료 판정');axs[0,0].legend(fontsize=9,loc='upper right')
colors=['#237ea5','#d18b27','#168a73'];off=[-.24,0,.24]
for sc,s in enumerate(summaries):
 x=np.arange(1,11)+off[sc]
 for ax,key in [(axs[0,1],'sat_percent'),(axs[1,0],'upper_failed')]:ax.bar(x,[z[key] for z in s['attempts']],width=.23,color=colors[sc],label=f'시나리오{sc+1}')
 axs[1,1].plot(np.arange(1,11),[z['lateral_abs']['p95'] for z in s['attempts']],'o-',color=colors[sc],label=f'시나리오{sc+1}',ms=4)
for ax,title,ylabel in [(axs[0,1],'회차별 출력 조향 명령 포화','%'),(axs[1,0],'주행 구간 상위 계획 실패 횟수','회'),(axs[1,1],'지역참조 횡오차 절대값 p95','m')]:ax.set_title(title);ax.set_ylabel(ylabel);ax.set_xticks(range(1,11));ax.set_xlabel('회차');ax.grid(axis='y',alpha=.2);ax.legend(fontsize=8)
fig.suptitle('최신30회 결과 비교 — 원인 분석은 보류',fontsize=17);fig.supxlabel('completed는 목표 접근이며 목표 정차 성공이 아님 · 조향은 명령값 · 경로·주행구간 차이가 있어 안정성 순위로 단정하지 않음',fontsize=10);fig.savefig(H/'results_comparison.png',dpi=150);plt.close(fig)
print('Three reports and result comparison saved')
