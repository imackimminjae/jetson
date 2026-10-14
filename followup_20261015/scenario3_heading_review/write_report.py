from pathlib import Path
import json,csv,pickle,hashlib
import numpy as np
from PIL import Image
P=Path(__file__).resolve().parent;ROOT=P.parents[1];R=json.loads((P/'metrics.json').read_text());S=json.loads((P/'summary.json').read_text());A=S[0]['all'];B=S[2]['all'];old=R[0];new=R[2]
lines=['시나리오 3 헤딩 비용 변경 분석 (2026-10-15)',
'', '가장 중요한 설정 확인',
'이번 기록: drive_batches/20261015_024902_scenario3_3778ac, 5회 모두 completed 및 stationary.',
'변경된 값은 하위 컨트롤러 lower_q_heading: 0.5 → 0.3.',
'상위 플래너 lambda_relative_turn는 기존과 동일하게 0.6. 시나리오1에서 추천한 상위 헤딩 비용 0.3의 검증 시험과 다르다.',
'주행 약 7.4초 전에 lower_q_heading=0.3 파라미터 이벤트 확인. 주행 중 핵심 튜닝 값 변경 없음.',
'플래너 5Hz, 컨트롤러 10Hz, 하위 예측 0.1초×30=3초, upper_r_dpsi=3, 조향 변화 비용 150 모두 동일.',
'대조 기록: drive_batches/20261015_010154_scenario3_f9c4a7, 기존 하위 헤딩 0.5, 5회.',
'상·하위 실행파일과 시나리오 경로의 manifest 해시는 동일. 실행/기록 스크립트는 이전 시험 이후 변경되었음.',
'', '판단',
'하위 헤딩 비용 0.3은 5회 모두 제어 추종을 유지했으나 조향 부드러움이나 전체 궤적 반복성의 개선은 관찰되지 않음.',
'현재 기록만으로는 하위 헤딩 비용 0.5 유지가 더 유리해 보임. 상위 헤딩 비용 0.3의 시나리오3 효과는 아직 판단할 수 없음.',
'', '주요 비교 (조향/헤딩 지표: 각 5회 중앙값; 궤적 편차: 동일 진행 위치에서 5회 전체 계산)',
'지표 | 기존 하위 0.5 | 이번 하위 0.3']
fields=[('출력 조향 보정량 [도/100m]','variation_deg_per_100m'),('조향 변화속도 P95 [도/초]','rate_p95_degps'),('빠른 조향 잔차 RMS [도]','rapid_residual_rms_deg'),('조향 한계 도달 비율 [%]','limit_ratio_pct'),('차량 yaw 가속도 RMS [도/초²]','yaw_accel_rms_degps2'),('local path 횡오차 RMS [m]','lateral_rmse_m'),('local path 헤딩 오차 RMS [도]','heading_rmse_deg'),('목표 접근 기록 시간 [초]','duration_sec')]
for label,k in fields:lines.append(f"{label} | {A[k]['median']:.3f} | {B[k]['median']:.3f}")
for label,k in [('전체 궤적 편차 RMS [m]','rms_spread_m'),('첫 회전 궤적 편차 RMS [m]','first_turn_rms_spread_m'),('두 번째 회전 궤적 편차 RMS [m]','second_turn_rms_spread_m'),('출구 궤적 편차 RMS [m]','exit_rms_spread_m')]:lines.append(f"{label} | {old['spatial'][k]:.3f} | {new['spatial'][k]:.3f}")
lines+=['', '해석',
f"- 조향 보정량 +{(B['variation_deg_per_100m']['median']/A['variation_deg_per_100m']['median']-1)*100:.1f}%, 조향 변화속도 P95 +{(B['rate_p95_degps']['median']/A['rate_p95_degps']['median']-1)*100:.1f}%, 실제 차량 yaw 가속도 +{(B['yaw_accel_rms_degps2']['median']/A['yaw_accel_rms_degps2']['median']-1)*100:.1f}%.",
'- 전체 반복 궤적 편차는 0.371→0.369m로 사실상 비슷함.',
'- 첫 회전은 반복성이 개선(0.541→0.343m)됐지만 두 번째 회전(0.349→0.504m)과 출구(0.175→0.276m)는 악화.',
'- 두 번째 회전의 조향 보정량 자체는 376.5→378.5도/100m로 비슷하나 한계 도달 비율은 19.74→33.78%로 증가.',
'- 출구 구간 보정량은 101.1→172.0도/100m로 약 70% 증가. 직선으로 빠져나오며 보정이 많아짐.',
'- local path 횡오차 중앙값 0.046→0.090m, 헤딩 오차 7.12→8.59도. 추종 정확도도 개선되지 않음.',
'- 하위 헤딩 비용은 방향 오차에 대한 벌점이며 조향 부드러움 자체의 비용이 아님. 낮춘다고 조향 변화가 반드시 감소하지 않음.',
'- 목표 접근 시간은 37.90→36.70초로 감소했지만 정차 성능이나 전체 안정성이 좋아졌다는 근거로 쓰지 않음.',
'', '각 회차 실제 결과',
'회차 | outcome | 보정량 도/100m | 변화속도P95 도/초 | yaw 가속도 도/초² | 상위 계획 실패/시도']
for a in new['attempts']:lines.append(f"{a['attempt']} | {a['outcome']} | {a['steering']['variation_deg_per_100m']:.1f} | {a['steering']['rate_p95_degps']:.1f} | {a['yaw_accel_rms_degps2']:.2f} | {a['upper_failures']}/{a['upper_attempts']}")
lines+=['', '제어 연속성과 상위 계획 실패',
'- 이번 5회: 이동 중 중립(mode3), 보호 정지(mode4), 오류 대체 제어(mode1/2/5) 없음. mode4는 각 회차 시작 직후 정지 상태 한 샘플만.',
'- BEV 최대 수신 간격 0.224초, pose 0.052초, 하위 제어 0.106초. 긴 입력 공백 및 제어 기록 공백 없음.',
'- 상위 전체 계산 최대 15.66ms, 하위 전체 계산 최대 4.62ms. 설정 주기 200/100ms 초과 없음.',
'- 상위 계획 실패는 기존 23/846(2.72%) → 이번 34/818(4.16%). 계산시간 부족이 아닌 corridor QP infeasible.',
'- 이번 실패는 대체로 19~23초에 발생, 연속 최대 6회/약 1.2초 후 회복. 이전 경로로 추종을 이어가 1.5초 path timeout에 도달하지 않음.',
'- 상위 비용은 동일해도 하위 제어로 달라진 자세·입력 상태가 이후 계획에 영향을 주므로, 실패 증가 원인을 상위 비용 변경이라고 해석하지 않음.',
'', '거리 정규화 감도 확인',
'- 기존 분석과 동일하게 하위 trace의 속도를 적분해 보정량을 주행 거리로 정규화한 값이 주 지표.',
'- 기록된 pose 궤적 길이로 대신 정규화해도 238.9→260.3도/100m(+9.0%)로 보정량 증가 방향은 유지.',
'- 증가율은 거리 추정 방법에 따라 약 9~16%. 실제 heading 거칠기 및 조향 변화속도 지표는 거리 추정과 별개로 증가.',
'', '이번 비교의 범위',
'- 각 5회 모두 평가했으며 회차 제외 없음. 참고용 이전 플래너2Hz 기록도 metrics.json에 별도로 보존.',
'- 순차 실행한 두 세션의 관찰 결과이며 반복 횟수 5회만으로 인과관계나 통계적 우위를 확정하지 않음.',
'- 궤적 편차는 같은 웨이포인트 진행 위치에서 반복 궤적이 얼마나 벌어지는지 측정하며 도로 중심선 오차가 아님.',
'- running 구간만 평가. completed는 목표 약 8m 접근 판정으로 목표 정차 성공이 아님. 이번 실제 정차 목표거리 34.0~36.4m.',
'- road evidence unavailable이므로 completed를 도로 경계 준수 검증으로 해석하지 않음.',
'', '앞선 시나리오1 추천과 같은 설정을 시험하려면',
'upper_planner_node.lambda_relative_turn = 0.3',
'lower_tracking_mpc_node.lower_q_heading = 0.5',
'플래너 5Hz / 제어 10Hz / 예측 3초 및 나머지 기준값 유지. 본 분석에서는 실행 설정을 변경하거나 새 주행을 시작하지 않음.',
'', '평가 방법',
'- 실제 송신 MAVLink 조향(index17×index31)을 각도로 환산. 조향 센서의 실측값이 아님.',
'- 속도 1m/s 초과에서 인접 명령 변화량 절대값 합을 이동거리로 정규화. 조향속도는 실제 명령 timestamp 차이 사용.',
'- 빠른 조향 잔차는 공통100Hz 유지 명령에서 0.5초 이동평균을 제거. 이전 보고서의10Hz 보간과 달라 해당 잔차 지표는 양쪽 모두 재계산.',
'- pose yaw는 기록 형식대로 scalar yaw 사용, 공통20Hz 시간격자와 9샘플 Savitzky-Golay 미분으로 yaw 가속도 RMS 계산.',
'- 궤적은 웨이포인트 진행거리 5~154.5m, 0.5m 간격 정렬. 첫 회전35~80m, 두 번째105~145m, 출구145~155m.',
'- 선택 토픽 수 metadata 대조, 설정/런타임 이벤트/실행파일 및 경로 manifest 비교, 원본 SHA256 종료 후 재확인.',
'', '산출물: overview.png, trajectories.png, spatial_signals.png, scenario3_heading_comparison.pdf, per_attempt.csv, set_comparison.csv, metrics.json, summary.json, distance_sensitivity.json, validation.json.']
(P/'REPORT.txt').write_text('\n'.join(lines)+'\n')
rows=[]
for r,s in zip(R,S):
 e=s['all'];rows.append(dict(set=r['set'],completed=s['successes'],runs=s['runs'],steering_variation_median_deg_per_100m=e['variation_deg_per_100m']['median'],steering_rate_p95_median_degps=e['rate_p95_degps']['median'],yaw_acceleration_median_degps2=e['yaw_accel_rms_degps2']['median'],spread_rms_m=r['spatial']['rms_spread_m'],first_turn_spread_m=r['spatial']['first_turn_rms_spread_m'],second_turn_spread_m=r['spatial']['second_turn_rms_spread_m'],exit_spread_m=r['spatial']['exit_rms_spread_m'],upper_failures=e['upper_failures_total'],upper_attempts=e['upper_attempts_total']))
with (P/'set_comparison.csv').open('w',newline='') as f:w=csv.DictWriter(f,fieldnames=rows[0]);w.writeheader();w.writerows(rows)
sources=json.loads((P/'sources_sha256.json').read_text())
assert all(hashlib.sha256(Path(p).read_bytes()).hexdigest()==h for p,h in sources.items())
for p in P.glob('*.png'):
 with Image.open(p) as im:im.verify()
# Verify the changed parameter and same core executables/route from frozen manifests.
import yaml
co=yaml.safe_load((ROOT/'drive_batches'/old['id']/'effective_config.yaml').read_text());cn=yaml.safe_load((ROOT/'drive_batches'/new['id']/'effective_config.yaml').read_text())
assert cn['upper_planner_node']['ros__parameters']['lambda_relative_turn']==.6
assert co['lower_tracking_mpc_node']['ros__parameters']['lower_q_heading']==.5
assert cn['lower_tracking_mpc_node']['ros__parameters']['lower_q_heading']==.3
mo=json.loads((ROOT/'drive_batches'/old['id']/'manifest.json').read_text());mn=json.loads((ROOT/'drive_batches'/new['id']/'manifest.json').read_text())
for suffix in ['upper_planner_node','lower_tracking_mpc_node','scenario3.csv']:
 ho=next(v for k,v in mo['sha256'].items() if k.endswith('/'+suffix));hn=next(v for k,v in mn['sha256'].items() if k.endswith('/'+suffix));assert ho==hn
for r in R:
 assert len(r['attempts'])==5 and all(a['outcome']=='completed' for a in r['attempts'])
 for a in r['attempts']:assert np.isfinite(a['steering']['variation_deg_per_100m'])
v=json.loads((P/'validation.json').read_text());v.update(source_file_count=len(sources),original_hashes_rechecked_unchanged=True,png_verified=True,new_session_completed=5,excluded_attempts=[],upper_heading_cost=.6,changed_parameter='lower_q_heading',old_value=.5,new_value=.3,core_executables_and_route_manifest_hashes_equal=True,session_scripts_changed_since_baseline=True)
(P/'validation.json').write_text(json.dumps(v,ensure_ascii=False,indent=2)+'\n')
print('Report, CSV, validation saved:',len(sources),'source files rechecked unchanged')
