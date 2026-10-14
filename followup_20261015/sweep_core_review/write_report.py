from pathlib import Path
import csv,json,pickle,hashlib
import numpy as np
from PIL import Image
P=Path(__file__).resolve().parent;R=json.loads((P/'metrics.json').read_text());S=json.loads((P/'summary.json').read_text());sens=json.loads((P/'sensitivity.json').read_text());ROOT=P.parents[1]
base=S[0]['all'];soft=S[4]['all'];c20=S[3]['all']
lines=['시나리오 1: 파라미터 9세트 × 3회 분석 (2026-10-15)',
'대상: parameter_sweeps/scenario1_core_3runs_20261015; 27회 모두 평가, 사용자 지정 제외 회차 없음.',
'결과: 목표 접근 기준 completed 26회, off_route 1회, 모든 회차 stationary 확인.',
'추천: 세트 5 heading_soft. 플래너 5Hz / 제어 10Hz / 하위 예측 3초 / lambda_relative_turn=0.3.',
'그 외 유지: upper_r_dpsi=3, 상위 조향 한계 0.30rad, 하위 헤딩 비용 0.5, 하위 조향 변화 비용 150.',
'세트 3 플래너 10Hz는 비용을 기준값으로 유지할 경우 유력한 대안.',
'', '전체 비교 (조향 및 차량 헤딩 거칠기: 개별 회차 지표 중앙값; 궤적 편차: 위치 정렬 후 회차 간 RMS)',
'세트 | completed | 조향 보정 도/100m [최소~최대] | yaw 가속도 RMS 도/초² | 궤적 편차 m | 상위 계획 실패/시도']
for j,(r,s) in enumerate(zip(R,S),1):
 a=s['all'];tv=a['variation_deg_per_100m'];sp=r['spatial'];lines.append(f"{j}. {r['label']} | {s['successes']}/3 | {tv['median']:.1f} [{tv['min']:.1f}~{tv['max']:.1f}] | {a['yaw_accel_rms_degps2']['median']:.2f} | {sp['rms_spread_m']:.3f}"+(' (완주 2회만)' if s['successes']<3 else '')+f" | {a['upper_failures_total']}/{a['upper_attempts_total']}")
lines+=['', '해석',
f"- 세트 5: 기준 대비 조향 보정량 {(1-soft['variation_deg_per_100m']['median']/base['variation_deg_per_100m']['median'])*100:.1f}% 감소, 실제 헤딩 가속도 {(1-soft['yaw_accel_rms_degps2']['median']/base['yaw_accel_rms_degps2']['median'])*100:.1f}% 감소.",
f"  회차 간 궤적 편차는 0.384→0.086m. 3회 모두 목표 접근, 상위 계획 실패 0회. 조향 한계 도달 비율 중앙값 0%.",
'  회전부 조향 보정량 중앙값 221.6→150.1도/100m; 출구 구간 43.0→13.6도/100m.',
'  하위가 계획 경로를 따라가는 횡오차 RMS 중앙값은 0.046→0.058m로 소폭 증가. 절대 도로 중심선 정확도와 다른 지표.',
'- 세트 3: 플래너 10Hz는 기준과 조향 보정량이 비슷하지만 궤적 편차 0.384→0.136m로 개선.',
'- 세트 2: 플래너 2Hz는 조향 보정량 114.3도/100m, 출구 구간 104.8도/100m. 갱신을 늦춘다고 전체가 부드러워지지 않음.',
'  3회차 30.40~31.40초에 stale lower path로 약 1초 중립 제어. 상위 실패 후 이전 경로 유지가 1.5초 유효기간을 넘김.',
f"- 세트 4: 제어 20Hz에서 예측 3초를 유지해도 조향 보정량 +{(c20['variation_deg_per_100m']['median']/base['variation_deg_per_100m']['median']-1)*100:.1f}%, 헤딩 가속도 +{(c20['yaw_accel_rms_degps2']['median']/base['yaw_accel_rms_degps2']['median']-1)*100:.1f}%.",
'  궤적 편차는 0.122m로 작지만 조향과 차량 움직임이 거칠어짐. 계산시간 초과는 아님.',
'  코드상 lower_rd_steering_rate는 초당 변화가 아닌 인접 예측 조향의 차이를 벌점으로 부과하며 dt로 나누지 않음.',
'  따라서 dt를 절반으로 줄이고 N을 두 배로 하면 같은 150도 물리적으로 같은 완화 강도가 아님.',
'  연속시간 비용의 근사로는 상대 변화율 완화 강도가 약 1/4이 되며, 20Hz에서 600을 검토할 수 있으나 이 조합은 미시험.',
'- 세트 6: 헤딩 비용 1.2는 완주했으나 부드러움이 기준과 비슷하고 0.3보다 반복성이 나쁨.',
'- 세트 7: upper_r_dpsi=6에서 1회 이탈. 정상 완주한 2회도 조향 보정량 109.3/129.2도/100m로 이점 없음.',
'- 세트 8: 상위 조향 한계 0.25rad는 한계 도달 비율을 줄이지만 전체 보정량은 기준과 비슷하고 상위 계획 실패 11회로 가장 많음.',
'  upper_r_dpsi=3 유지, normalize_upper_input_cost=false이므로 이번 시험에서는 비용 정규화의 동시 변화 없음.',
'- 세트 9: 하위 조향 변화 비용 300은 이름처럼 무조건 부드러워지지 않음. 보정량 104.0, 궤적 편차 0.496m로 개선 없음.',
'', '세트 7 1회차 이탈의 관측 원인',
'- 목표 출구는 동쪽(웨이포인트 118,-15 → 173,-2)인데 약 26.1초 이후 상위 계획이 북쪽으로 향함.',
'- 25.7초 계획 끝점 약 (129.2,-5.9)에서 26.1초 (124.4,1.0), 27.7초 (125.6,8.0)로 바뀜.',
'- 26~28초 차량 헤딩이 약 69→90도로 바뀌며 목표 출구 방향으로의 회전을 되돌림. 이후 회복 못하고 이탈.',
'- BEV 최대 수신 간격 0.161초, pose 최대 0.048초, 하위 제어 간격 최대 0.104초.',
'- 회전 중 하위 모드 0(정상 QP) 지속, 보호 정지나 fallback 없음. 상위 계획 실패는 25.5/28.5/28.9초에 3회.',
'- 하위는 살아 있는 계획을 계속 따라갔으므로 계획 방향/도로 후보 선택 문제가 직접 관측됨.',
'- 비용 증가가 유발 원인이라는 인과관계나 특정 후보 생성 버그까지 이 3회 기록만으로 확정할 수 없음.',
'', '입력과 실행 연속성',
f"- 27회 BEV 최대 수신 간격은 {max(a['grid_gap_max_s'] for r in R for a in r['attempts']):.3f}초. 0.5초 이상 BEV 중단 없음.",
'- 세트 5 2회차 4.43~5.10초에 하위 trace 간격 0.674초; stale pose 중립 1회(약 0.1초).',
'- 세트 9 1회차 4.54~5.22초에 하위 trace 간격 0.686초; stale pose 중립 1회(약 0.1초).',
'- 두 구간 BEV, pose, run_permission 수신은 계속됨. 하위 노드 실행/콜백 지연과 일치하며 정확한 지연 원인은 기록만으로 미확정.',
'- 이 회차도 제외하지 않음. 제어 기록이 없는 구간의 실제 중간 출력은 추정하지 않음.',
'- 감도 확인: 진행 15~175m로 초반을 제외해도 기준 조향 112.6 vs 세트 5 76.6도/100m, yaw 가속도 9.04 vs 6.83.',
'- 긴 기록 간격을 포함해 끝점 명령 변화량을 계산하는 대안에서도 세트 5 70.1 vs 기준 100.6으로 결론 유지.',
'- 상위 전체 계산 최대 약 21.1ms, 하위 전체 계산 최대 약 9.2ms; 설정 주기를 넘는 계산 0회.',
'  단, 계산시간 지표는 콜백 사이의 실행 공백과 다르므로 0.67초 공백의 부재를 의미하지 않음.',
'', '목표 접근과 실제 정차는 분리할 것',
'- completed는 목표 약 7.5~8.0m 접근 시 waypoint guard의 판정이며 정차 위치 정확도 판정이 아님.',
'- 완주한 26회 모두 목표의 동쪽을 지나 x≈206~212m에서 정차; 목표(173,-2)까지 정차 거리 33.4~38.8m.',
'- 따라서 본 조향/궤적 비교는 running 구간까지만 평가. 목표 정차 정확도가 요구된다면 정차 출력/감속은 별도 개선 대상.',
'- road evidence는 unavailable이므로 26회의 completed를 도로 경계 준수의 검증으로 해석하지 않음.',
'', '계산 방법과 한계',
'- 실제 보낸 MAVLink 조향(index17 × index31)의 변화량을 주행 거리로 나눈 TV(도/100m). 초기 저속 speed<=1m/s 및 coasting 제외.',
'- 부드러움 지표는 낮을수록 좋지만 조향을 안 하면 낮아질 수 있으므로 완주 여부와 궤적 반복성을 함께 평가.',
'- 차량 헤딩은 기록된 scalar yaw를 사용. 20Hz 공통 시간 격자, 0.45초 Savitzky-Golay 기울기 후 yaw 가속도 RMS 계산.',
'- 궤적 편차는 동일 웨이포인트 진행 거리 5~179.5m를 0.5m 간격으로 정렬하여 회차 평균으로부터의 2D RMS. 정확한 도로 중심선 오차가 아님.',
'- 세트 7의 전체 경로 편차는 완주한 2회만 가능. 이탈을 모든 조향/헤딩 지표와 그림에 보존하고 성공 2/3으로 표시.',
'- 다른 세트끼리 한 번의 주행 시간으로 섞지 않고 각 회차를 같은 비중으로 중앙값 비교.',
'- 3회 표본, 순차 세트 실행, 조건의 확률적 변화가 있으므로 우열의 확정이나 다른 시나리오 일반화는 불가.',
'- 플래너 10Hz + 헤딩 비용 0.3 조합은 아직 미시험. 현재 가장 직접 근거가 있는 추천은 실제 시험한 세트 5.',
'', '검증: 9개 bag 선택 토픽 수를 metadata와 대조, effective_config와 준비 설정 완전 일치, 주행 중 핵심 튜닝 파라미터 변경 없음.',
'원본 파일 SHA256 저장 및 종료 후 재확인. 실행 설정/원본 로그를 변경하지 않음.',
'첨부: overview.png, trajectories.png, spatial_signals.png, failure_detail.png, sweep_comparison.pdf, per_attempt.csv, metrics.json, sensitivity.json.']
(P/'REPORT.txt').write_text('\n'.join(lines)+'\n')
# Table used in downstream comparisons, with failed trajectories explicitly retained.
rows=[]
for r,s in zip(R,S):
 a=s['all'];rows.append(dict(set=r['set'],completed=s['successes'],runs=s['runs'],steering_variation_median_deg_per_100m=a['variation_deg_per_100m']['median'],steering_variation_min=a['variation_deg_per_100m']['min'],steering_variation_max=a['variation_deg_per_100m']['max'],yaw_acceleration_median_degps2=a['yaw_accel_rms_degps2']['median'],completed_spread_rms_m=r['spatial']['rms_spread_m'],spread_attempts=len(r['spatial']['completed_attempts']),upper_failures=a['upper_failures_total'],upper_attempts=a['upper_attempts_total']))
with (P/'set_comparison.csv').open('w',newline='') as f:w=csv.DictWriter(f,fieldnames=rows[0]);w.writeheader();w.writerows(rows)
# Validate immutable input integrity and generated outputs.
src=json.loads((P/'sources_sha256.json').read_text());changed=[s for s,h in src.items() if hashlib.sha256(Path(s).read_bytes()).hexdigest()!=h];assert not changed,changed
for p in P.glob('*.png'):
 with Image.open(p) as im:im.verify()
for r in R:
 assert len(r['attempts'])==3
 for a in r['attempts']:
  assert a['steering']['travel_m']>100
  assert all(np.isfinite(a['steering'][k]) for k in ['variation_deg_per_100m','rate_p95_degps','rapid_residual_rms_deg'])
assert sum(a['outcome']=='completed' for r in R for a in r['attempts'])==26
v=json.loads((P/'validation.json').read_text());v.update(source_hashes_rechecked_unchanged=True,source_file_count=len(src),pngs_verified=True,attempts=27,completed=26,failed_attempts_preserved=1,excluded_attempts=[])
(P/'validation.json').write_text(json.dumps(v,ensure_ascii=False,indent=2)+'\n')
print('Report and CSV saved. Source files unchanged:',len(src))
