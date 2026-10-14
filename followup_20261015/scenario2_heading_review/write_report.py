from pathlib import Path
import json,csv,pickle,hashlib,yaml
import numpy as np
from PIL import Image
P=Path(__file__).resolve().parent;ROOT=P.parents[1];R=json.loads((P/'metrics.json').read_text());S=json.loads((P/'summary.json').read_text());sens=json.loads((P/'sensitivity.json').read_text());old,new=R;A,B=S[0]['all'],S[1]['all']
lines=['시나리오 2 최신 시험 분석 (2026-10-15)',
'결론: 추천 설정(플래너5Hz / 제어10Hz / 상위 헤딩0.3 / 하위 헤딩0.5)에서 목표 접근4/5, 3회차 경로 이탈.',
'조향 보정량은 이전 시험보다 줄었으나 갈림길 이탈이 남아 있어 시나리오2의 안정성이 확보되었다고 평가할 수 없음.',
'이탈은 입력 중단이나 하위 제어 정지보다 잘못된 방향의 상위 계획 경로를 따라간 흐름과 일치.',
'', '비교 조건과 제한',
f"이전: drive_batches/{old['id']}, 플래너2Hz / 상위 헤딩0.6 / 하위 헤딩0.5, completed4 / off_route1(2회차).",
f"이번: drive_batches/{new['id']}, 플래너5Hz / 상위 헤딩0.3 / 하위 헤딩0.5, completed4 / off_route1(3회차).",
'두 세션의 실제 route 파일 동일: (-250,101) → (-209,86) → (-201,74) → (-133,56) → (-76,67).',
'제어10Hz, 하위 예측3초, upper_r_dpsi3, 조향 한계0.3rad, 조향 변화 비용150 유지.',
'주파수와 상위 헤딩 비용이 함께 변경됐으므로 헤딩 비용만의 효과로 분리할 수 없음. 같은 경로/5Hz/상위0.6 대조 기록 없음.',
'상·하위 실행파일 manifest 해시는 동일. 실행/기록 스크립트는 이전 시험 이후 변경됨.',
'다른 두 이전 시나리오2 세션은 목표가 (-4,95)까지 더 긴 경로여서 직접 비교에서 제외. 원본은 보존.',
'', '전체5회 지표 중앙값 (이탈 회차도 포함)',
'지표 | 이전2Hz/0.6 | 이번5Hz/0.3']
fields=[('조향 보정량 도/100m','variation_deg_per_100m'),('조향 변화속도P95 도/초','rate_p95_degps'),('차량 yaw 가속도RMS 도/초²','yaw_accel_rms_degps2'),('빠른 조향 잔차RMS 도','rapid_residual_rms_deg'),('조향 한계 근접 시간비 %','limit_ratio_pct'),('local path 횡오차RMS m','lateral_rmse_m'),('local path 헤딩 오차RMS 도','heading_rmse_deg')]
for label,k in fields:lines.append(f"{label} | {A[k]['median']:.3f} | {B[k]['median']:.3f}")
lines+=['', '완주4회만의 조건부 지표 (실패는 앞선 지표와 실패 분석에 별도 보존)',
'지표 | 이전 | 이번']
for label,k in fields[:3]:lines.append(f"{label} | {S[0]['completed'][k]['median']:.3f} | {S[1]['completed'][k]['median']:.3f}")
for label,k in [('전체 반복 궤적 편차RMS m','rms_spread_m'),('첫 회전 편차RMS m','first_turn_rms_spread_m'),('두 번째 회전 편차RMS m','second_turn_rms_spread_m'),('출구 편차RMS m','exit_rms_spread_m')]:lines.append(f"{label} | {old['spatial'][k]:.3f} | {new['spatial'][k]:.3f}")
lines+=['', '이번 각 회차',
'회차 | outcome | 기록시간 초 | 조향 도/100m | yaw 가속도 도/초² | 상위 계획 실패']
for a in new['attempts']:lines.append(f"{a['attempt']} | {a['outcome']} | {a['duration_sec']:.3f} | {a['steering']['variation_deg_per_100m']:.1f} | {a['yaw_accel_rms_degps2']:.2f} | {a['upper_failures']}")
lines+=['', '해석',
'- 전체5회 조향 보정량161.8→139.4도/100m(약14% 감소). 완주4회만 비교해도155.7→136.7(약12% 감소).',
'- 완주4회의 실제 차량 yaw 가속도는9.81→9.78로 거의 동일. 명령 보정 감소가 차량 움직임의 큰 개선으로 이어졌다는 근거는 약함.',
'- 완주4회의 궤적 편차는0.307→0.561m로 증가. 주로 첫 회전0.426→1.060m의 차이.',
'- 이 값에는 초반 제어 공백이 있었던 이번1회차 영향이 큼. 1회차 제외 감도는0.324m로 이전 전체 완주4회0.307m에 가까움.',
'- 공식 비교에는1회차를 계속 포함. 공백이 궤적 차이의 유일 원인이라고 확정하지 않음.',
'- 실패 회차를 포함한 첫 갈림길 이전 공통5~114.5m 반복편차도 이전0.364 / 이번0.653m. 실패 제외만으로 좋아 보이게 만들지 않음.',
'- pose 궤적 길이로 거리 정규화를 바꾼 감도에서도 전체5회160.2→143.9도/100m, 완주4회154.6→142.6으로 보정량 감소 방향 유지.',
'', '3회차 이탈의 직접 관측',
'- 두 번째 갈림길(-133,56) 뒤 목표 경로는 (-76,67), 약+10.9도 방향(동쪽 위).',
'- 약23~24초부터 새 상위 계획이 동쪽 아래로 이어지고 차량 헤딩이 음의 방향으로 회전.',
'- 24.1초 차량(-138.4,55.9), 새 계획 끝점(-120.3,46.4). 28.1초 새 계획 끝점(-105.3,34.5).',
'- 하위는 정상 QP 모드0으로 해당 계획을 계속 따라갔고, 종료 판정에서 진행76.4%/횡이탈26.8m.',
'- 상위 QP infeasible은26.1초1회만 발생. 잘못된 방향의 계획은 그보다 앞서 보이므로 단발 solver 실패가 이탈 시작 원인은 아님.',
'- 실패 회차 BEV 최대 간격0.231초, pose 약0.05초 이내, 하위 제어 최대약0.104초. 목표도달 false, 실행허용 true 유지(판정 직후 해제).',
'- 이전2Hz/상위0.6의2회차도 같은 갈림길에서 아래쪽 방향으로 계획 및 실제 궤적이 진행하다 이탈.',
'- 분기 후보 선택/출구 방향 유도가 반복적으로 취약하다는 관측. 비용0.3 하나가 원인이라고 단정할 수 없고, 내부 후보 생성 버그까지 확정한 것은 아님.',
'', '1회차 초반 제어 공백',
'- running 시작2.77~3.54초 사이 하위 trace 약0.776초 공백. 복귀 후 stale pose로 중립(mode3) 한 샘플약0.1초.',
'- 그동안 BEV, pose, run_permission, 상위 계획 기록은 이어짐. 하위 실행/콜백 지연과 일치하나 정확한 지연 원인은 미확정.',
'- 이 현상은3회차 이탈과 별개이며, 이 회차는 이후 제어를 회복해 완주.',
'- 이번 전체 BEV 최대 간격0.231초, pose0.053초. 입력 송신 전체가 끊긴 것으로 볼 근거 없음.',
'- 상위 전체 계산 최대12.91ms, 하위 전체 계산 최대3.26ms. 설정 주기200/100ms 초과0회. 콜백 사이 공백과 계산시간은 별도 지표.',
'', '다음 확인 우선순위',
'- 시나리오2의 두 번째 갈림길에서 목표 출구 후보가 존재하는지, 후보 선택과 방향 비용이 어떻게 작동하는지 확인하는 것이 우선.',
'- 이번 로그는 잘못된 계획도 하위가 정상 추종할 수 있음을 보여줌. completed 비율과 local path 추종오차만으로 안정성을 판단하지 않음.',
'- 비용 효과를 독립 비교하려면 동일 경로/플래너5Hz에서 상위0.6과0.3의 대조가 필요. 본 요청에서는 새 주행이나 설정 변경을 수행하지 않음.',
'', '평가 방법과 한계',
'- 각5회 전체를 보존. 동일 완주 길이 궤적 편차는 각 완주4회만 계산했으며 조건부 수치임을 표시.',
'- completed는 목표 약8m 접근 판정으로 정차 정확도나 도로 경계 준수의 검증이 아님. 평가 구간은running, coasting 제외.',
'- 실제 송신 MAVLink 조향index17×index31 환산. speed>1m/s, 정상 시간간격의 변화량 합을 주행 거리로 정규화.',
'- scalar yaw를 공통20Hz와9샘플 Savitzky-Golay 미분으로 분석. 조향 빠른 잔차는공통100Hz 유지 명령의0.5초 이동평균 잔차.',
'- 반복 궤적 공통거리5~174.5m, 간격0.5m. 첫 회전30~75m, 두 번째115~150m, 출구150~175m.',
'- 궤적 편차는 회차 평균에서의2D RMS이며 도로 중심선 오차가 아님. local path 오차는 계획 경로 갱신 빈도의 영향을 받음.',
'- 순차 시험 각5회. 주파수/비용 동시 변경, 실행시각과 입력 조건의 차이로 인과관계나 통계적 우위 확정 불가.',
'- 선택 토픽 수 metadata 대조, 설정/파라미터 이벤트 확인, 원본SHA256 종료 후 재확인. 운영 코드/설정/로그 미변경.',
'', '산출물: overview.png, trajectories.png, spatial_signals.png, failure_detail.png, scenario2_analysis.pdf, per_attempt.csv, set_comparison.csv, metrics.json, sensitivity.json, validation.json.']
(P/'REPORT.txt').write_text('\n'.join(lines)+'\n')
rows=[]
for r,s in zip(R,S):
 for population in ['all','completed']:
  e=s[population];rows.append(dict(set=r['set'],population=population,completed=s['successes'],attempts=s['runs'] if population=='all' else s['successes'],steering_variation_median_deg_per_100m=e['variation_deg_per_100m']['median'],steering_rate_p95_median_degps=e['rate_p95_degps']['median'],yaw_acceleration_median_degps2=e['yaw_accel_rms_degps2']['median'],completed_spread_rms_m=r['spatial']['rms_spread_m'],spread_attempts=len(r['spatial']['completed_attempts']),upper_failures=e['upper_failures_total'],upper_attempts=e['upper_attempts_total']))
with (P/'set_comparison.csv').open('w',newline='') as f:w=csv.DictWriter(f,fieldnames=rows[0]);w.writeheader();w.writerows(rows)
sources=json.loads((P/'sources_sha256.json').read_text());assert all(hashlib.sha256(Path(p).read_bytes()).hexdigest()==h for p,h in sources.items())
for p in P.glob('*.png'):
 with Image.open(p) as im:im.verify()
co=yaml.safe_load((ROOT/'drive_batches'/old['id']/'effective_config.yaml').read_text());cn=yaml.safe_load((ROOT/'drive_batches'/new['id']/'effective_config.yaml').read_text());changes=[]
for node,c in co.items():
 a=c['ros__parameters'];b=cn[node]['ros__parameters']
 for k in a.keys()|b.keys():
  if a.get(k)!=b.get(k):changes.append((node,k))
assert set(changes)=={('upper_planner_node','lambda_relative_turn'),('upper_planner_node','upper_planner_rate_hz'),('knu_batch_runner','results_directory')},changes
for k,expected in [('lambda_relative_turn',.3),('upper_planner_rate_hz',5.)]:assert cn['upper_planner_node']['ros__parameters'][k]==expected
assert cn['lower_tracking_mpc_node']['ros__parameters']['lower_q_heading']==.5
assert (ROOT/'drive_batches'/old['id']/'routes/scenario2.csv').read_bytes()==(ROOT/'drive_batches'/new['id']/'routes/scenario2.csv').read_bytes()
mo=json.loads((ROOT/'drive_batches'/old['id']/'manifest.json').read_text());mn=json.loads((ROOT/'drive_batches'/new['id']/'manifest.json').read_text())
for suffix in ['upper_planner_node','lower_tracking_mpc_node']:
 assert next(v for k,v in mo['sha256'].items() if k.endswith('/'+suffix))==next(v for k,v in mn['sha256'].items() if k.endswith('/'+suffix))
critical={'lambda_relative_turn','lower_q_heading','upper_planner_rate_hz','lower_controller_rate_hz','upper_r_dpsi','lower_rd_steering_rate','max_steering_angle_rad'}
d=pickle.loads((P/f"decoded_{new['id']}.pkl").read_bytes())
for t,node,vals in d['data']['/parameter_events']:
 if any(a['start']<=t<a['end'] for a in new['attempts']):assert not any(k in critical for k,v in vals),(t,node,vals)
assert len(new['attempts'])==5 and [a['outcome'] for a in new['attempts']]==['completed','completed','off_route','completed','completed']
assert all(np.isfinite(a['steering']['variation_deg_per_100m']) for r in R for a in r['attempts'])
v=json.loads((P/'validation.json').read_text());v.update(source_file_count=len(sources),original_hashes_rechecked_unchanged=True,png_verified=True,current_session_completed=4,current_session_failed=1,failed_attempts_preserved=True,excluded_attempts=[],compared_control_changes=['upper_planner_rate_hz 2→5','lambda_relative_turn 0.6→0.3'],lower_heading_cost=.5,core_executables_and_route_equal=True,no_critical_tuning_changes_during_current_runs=True,repeatability_is_conditional_on_completed_runs=True)
(P/'validation.json').write_text(json.dumps(v,ensure_ascii=False,indent=2)+'\n')
print('Report and data validated;',len(sources),'source files unchanged')
