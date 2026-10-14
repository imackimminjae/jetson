from pathlib import Path
import json,csv,pickle,hashlib,yaml
import numpy as np
from PIL import Image
P=Path(__file__).resolve().parent;ROOT=P.parents[1];R=json.loads((P/'metrics.json').read_text());S=json.loads((P/'summary.json').read_text());old=R[0];new=R[2];A=S[0]['all'];B=S[2]['all'];sens=json.loads((P/'distance_sensitivity.json').read_text())
lines=['시나리오 3: 상위 헤딩 비용 0.6 → 0.3 재시험 분석 (2026-10-15)',
'결론: 상위 lambda_relative_turn=0.3, 하위 lower_q_heading=0.5가 적용된 5회 모두 목표 접근 및 정지 확인.',
'기준보다 조향 보정량과 변화속도가 감소하고 반복 궤적이 일정해졌음. 시나리오1에서도 같은 설정이 유리했으므로 현재 기본 후보로 추천.',
'출구 구간의 짧은 조향 보정 및 local path 횡오차 증가가 남아 있어 모든 지표가 개선된 것은 아님.',
'', '실제 비교 조건',
f"기준: drive_batches/{old['id']}, 상위 헤딩 0.6 / 하위 헤딩 0.5 / 5회.",
f"이번: drive_batches/{new['id']}, 상위 헤딩 0.3 / 하위 헤딩 0.5 / 5회.",
'플래너 5Hz, 제어10Hz, 하위 예측0.1초×30=3초, upper_r_dpsi=3, 조향 한계0.3rad, 하위 조향 변화 비용150 유지.',
'실행 당시 설정의 변경은 lambda_relative_turn과 결과 저장 위치뿐. 준비 파라미터 이벤트에서 상위0.3/하위0.5 확인, 주행 중 핵심 튜닝 변경 없음.',
'상·하위 실행파일과 경로 manifest 해시는 동일. 실행/기록 스크립트는 이전 기준 시험 이후 변경되었음.',
'025626 세션은 하위 헤딩0.6 상태로 약16초 후 사용자 중단(batch_aborted). 다른 조건의 미완료 기록이므로 동일 조건 5회 비교에서 분리하며 제어 실패로 세지 않음.',
'직전 하위 헤딩0.3 시험은 참고 그룹으로 수치와 원본을 보존.',
'', '전체 비교: 각 회차 중앙값; 궤적 편차는 동일 진행 위치에서 5회 전체 계산',
'지표 | 기존 상위0.6 | 이번 상위0.3 | 변화']
for label,k in [('조향 보정량 도/100m','variation_deg_per_100m'),('조향 변화속도P95 도/초','rate_p95_degps'),('빠른 조향 잔차RMS 도','rapid_residual_rms_deg'),('차량 yaw 가속도RMS 도/초²','yaw_accel_rms_degps2'),('조향 한계 근접 시간비 %','limit_ratio_pct'),('local path 횡오차RMS m','lateral_rmse_m'),('local path 헤딩 오차RMS 도','heading_rmse_deg'),('목표 접근 기록 시간 초','duration_sec')]:
 a=A[k]['median'];b=B[k]['median'];lines.append(f'{label} | {a:.3f} | {b:.3f} | {(b/a-1)*100:+.1f}%')
for label,k in [('전체 반복 궤적 편차RMS m','rms_spread_m'),('첫 회전 편차RMS m','first_turn_rms_spread_m'),('두 번째 회전 편차RMS m','second_turn_rms_spread_m'),('출구 편차RMS m','exit_rms_spread_m')]:
 a=old['spatial'][k];b=new['spatial'][k];lines.append(f'{label} | {a:.3f} | {b:.3f} | {(b/a-1)*100:+.1f}%')
lines+=['', '구간별 조향 보정량 중앙값 (도/100m)']
segrows=[]
for name,label in [('approach','진입'),('first_turn','첫 회전'),('middle','중간'),('second_turn','두 번째 회전'),('exit','출구')]:
 a=float(np.median([v['segments'][name]['variation_deg_per_100m'] for v in old['attempts']]));b=float(np.median([v['segments'][name]['variation_deg_per_100m'] for v in new['attempts']]));lines.append(f'{label}: {a:.1f} → {b:.1f} ({(b/a-1)*100:+.1f}%)');segrows.append(dict(segment=name,baseline_deg_per_100m=a,upper_0_3_deg_per_100m=b,change_pct=(b/a-1)*100))
lines+=['', '해석과 남은 차이',
'- 개선의 중심은 회전부. 첫 회전 보정량 약25%, 두 번째 회전 약25% 감소.',
'- 전체 궤적 편차 약33% 감소, 첫 회전 약47% 감소, 두 번째 약11% 감소, 출구 약72% 감소.',
'- 출구 궤적은 더 일정해졌으나 조향 보정량은 101.1→147.3도/100m(약46% 증가). 반복 궤적의 일관성과 순간 명령 부드러움은 별개.',
'- local path 횡오차 중앙값 0.046→0.061m로 약1.5cm 증가. 도로 중심선 오차가 아닌 갱신된 계획 경로 대비 오차.',
'- 실제 차량 yaw 가속도 감소는 약5%로 비교적 작음. 조향 변화속도P95의 약20% 감소와 구분할 것.',
'- 조향 한계 근접 전체 중앙값은 4.63→4.43%로 큰 차이 없음. 두 번째 회전은 19.74→14.08%로 개선.',
'- 기준 조향 보정량 회차 범위198.1~297.4, 이번190.6~222.4도/100m. 이번 표본에서는 회차 편차도 작음.',
'', '제어 연속성 및 입력 상태',
'- 이번 5회 모두 이동 중 중립(mode3), 보호 정지(mode4), 대체 제어(mode1/2/5) 없음. 각 회차 초기 정지 상태에서 mode4 한 샘플만.',
'- BEV 최대 수신 간격0.248초, pose0.048초, 하위 제어0.104초. 0.5초 이상 입력/제어 기록 공백 없음.',
'- 상위 전체 계산 최대19.89ms, 하위 전체 계산 최대2.97ms. 설정 주기200/100ms를 넘는 계산 없음.',
'- 상위 계획 실패는 기존23/846(2.72%) → 이번26/818(3.18%). 상위 비용을 낮춰도 QP infeasible 문제 자체는 줄지 않음.',
'- 이번 최대 연속 실패6회, 약1.2초 후 회복. 이전 계획 경로를 유지하여1.5초 경로 유효기간 내 정상 추종 지속.',
'', '각 회차 수치',
'회차 | outcome | 보정량 도/100m | 변화속도P95 도/초 | yaw 가속도 도/초² | 상위 실패/시도']
for a in new['attempts']:lines.append(f"{a['attempt']} | {a['outcome']} | {a['steering']['variation_deg_per_100m']:.1f} | {a['steering']['rate_p95_degps']:.1f} | {a['yaw_accel_rms_degps2']:.2f} | {a['upper_failures']}/{a['upper_attempts']}")
poseold=float(np.median([a['pose_normalized_variation_deg_per_100m'] for a in sens[0]['attempts']]));posenew=float(np.median([a['pose_normalized_variation_deg_per_100m'] for a in sens[2]['attempts']]))
lines+=['', '感度 확인'.replace('感度','감도'),
f'- 거리 정규화를 하위 속도 적분 대신 pose 궤적 길이로 바꿔도 {poseold:.1f}→{posenew:.1f}도/100m({(posenew/poseold-1)*100:+.1f}%)로 감소 방향 유지.',
'- 조향 보정량 개선 크기는 거리 추정 방법에 따라 약7~10%. 변화속도와 yaw 거칠기, 공간 반복성은 거리 적분 방법과 별개.',
'', '권고와 해석 범위',
'- 현재 추천 후보: 플래너5Hz / 제어10Hz / lambda_relative_turn0.3 / lower_q_heading0.5 / upper_r_dpsi3 / 예측3초 / 조향 변화 비용150.',
'- 시나리오1과3 모두 기준 대비 유리한 관찰 결과가 있으나 다른 시나리오의 최적값이나 통계적 우위를 확정하지 않음.',
'- 이번 완주5회는 모두 평가. 앞서 수동 중단한 다른 조건의 세션은 별도 보존하며 경로 이탈이나 알고리즘 실패로 취급하지 않음.',
'- completed는 목표 약8m 접근 판정이며 목표 정차 성공이나 도로 경계 준수의 검증은 아님.',
'- 평가 구간은 running이며 목표 판정 이후 coasting/정차는 제외.',
'- 궤적 편차는 동일 웨이포인트 진행 거리에서 회차 평균으로부터의2D RMS로, 도로 중심선 정확도와 구분.',
'- 비교는 순차 실행한 각5회 시험. 표본 범위가 겹치므로 모든 회차에서0.3이 더 낫다는 의미는 아님.',
'', '방법',
'- 실제 송신 MAVLink 조향 index17×index31 환산, speed>1m/s. 시간 간격0.015~0.25초인 인접 명령 변화 절대합을 주행 거리로 정규화.',
'- 공통100Hz 유지 조향에서0.5초 이동평균을 뺀 잔차로 빠른 변화 계산.',
'- scalar yaw를 공통20Hz로 정렬하고9샘플 Savitzky-Golay 미분으로 실제 차량 yaw 가속도RMS 계산.',
'- 궤적 공통 진행거리5~154.5m, 간격0.5m. 첫 회전35~80m, 두 번째105~145m, 출구145~155m.',
'- 선택 토픽 수 metadata 대조, 실행 설정/파라미터 이벤트 확인, manifest 실행파일/경로 해시 동일성 확인, 원본SHA256 종료 후 재확인.',
'- 운영 코드/설정 및 원본 로그는 변경하지 않음.',
'', '산출물: overview.png, trajectories.png, spatial_signals.png, scenario3_upper_heading_comparison.pdf, per_attempt.csv, set_comparison.csv, segment_comparison.csv, metrics.json, summary.json, distance_sensitivity.json, interrupted_session.json, validation.json.']
(P/'REPORT.txt').write_text('\n'.join(lines)+'\n')
rows=[]
for r,s in zip(R,S):
 e=s['all'];rows.append(dict(set=r['set'],completed=s['successes'],runs=s['runs'],steering_variation_median_deg_per_100m=e['variation_deg_per_100m']['median'],steering_rate_p95_median_degps=e['rate_p95_degps']['median'],yaw_acceleration_median_degps2=e['yaw_accel_rms_degps2']['median'],spread_rms_m=r['spatial']['rms_spread_m'],first_turn_spread_m=r['spatial']['first_turn_rms_spread_m'],second_turn_spread_m=r['spatial']['second_turn_rms_spread_m'],exit_spread_m=r['spatial']['exit_rms_spread_m'],upper_failures=e['upper_failures_total'],upper_attempts=e['upper_attempts_total']))
for filename,rr in [('set_comparison.csv',rows),('segment_comparison.csv',segrows)]:
 with (P/filename).open('w',newline='') as f:w=csv.DictWriter(f,fieldnames=rr[0]);w.writeheader();w.writerows(rr)
sources=json.loads((P/'sources_sha256.json').read_text());assert all(hashlib.sha256(Path(p).read_bytes()).hexdigest()==h for p,h in sources.items())
for p in P.glob('*.png'):
 with Image.open(p) as im:im.verify()
co=yaml.safe_load((ROOT/'drive_batches'/old['id']/'effective_config.yaml').read_text());cn=yaml.safe_load((ROOT/'drive_batches'/new['id']/'effective_config.yaml').read_text())
assert co['upper_planner_node']['ros__parameters']['lambda_relative_turn']==.6
assert cn['upper_planner_node']['ros__parameters']['lambda_relative_turn']==.3
assert cn['lower_tracking_mpc_node']['ros__parameters']['lower_q_heading']==co['lower_tracking_mpc_node']['ros__parameters']['lower_q_heading']==.5
changes=[]
for node,c in co.items():
 a=c['ros__parameters'];b=cn[node]['ros__parameters']
 for k in a.keys()|b.keys():
  if a.get(k)!=b.get(k):changes.append((node,k))
assert set(changes)=={('upper_planner_node','lambda_relative_turn'),('knu_batch_runner','results_directory')},changes
mo=json.loads((ROOT/'drive_batches'/old['id']/'manifest.json').read_text());mn=json.loads((ROOT/'drive_batches'/new['id']/'manifest.json').read_text())
for suffix in ['upper_planner_node','lower_tracking_mpc_node','scenario3.csv']:
 assert next(v for k,v in mo['sha256'].items() if k.endswith('/'+suffix))==next(v for k,v in mn['sha256'].items() if k.endswith('/'+suffix))
critical={'lambda_relative_turn','lower_q_heading','upper_planner_rate_hz','lower_controller_rate_hz','upper_r_dpsi','lower_rd_steering_rate','max_steering_angle_rad'}
d=pickle.loads((P/f"decoded_{new['id']}.pkl").read_bytes())
for t,node,vals in d['data']['/parameter_events']:
 if any(a['start']<=t<a['end'] for a in new['attempts']):assert not any(k in critical for k,v in vals),(t,node,vals)
assert len(new['attempts'])==5 and all(a['outcome']=='completed' for a in new['attempts'])
assert all(np.isfinite(a['steering']['variation_deg_per_100m']) for r in R for a in r['attempts'])
v=json.loads((P/'validation.json').read_text());v.update(source_file_count=len(sources),original_hashes_rechecked_unchanged=True,png_verified=True,new_session_completed=5,new_session_excluded_attempts=[],other_session_separately_preserved='20261015_025626_scenario3_429ffb (user interrupted; lower heading0.6)',changed_control_parameter='lambda_relative_turn',old_value=.6,new_value=.3,lower_heading_cost=.5,core_executables_and_route_manifest_hashes_equal=True,no_critical_tuning_changes_during_new_runs=True)
(P/'validation.json').write_text(json.dumps(v,ensure_ascii=False,indent=2)+'\n')
print('Report and data validated;',len(sources),'source files unchanged')
