"""Correct/validate derived tables and write a concise Korean results report."""
from pathlib import Path
from collections import Counter
import csv,gzip,json,hashlib,html,tarfile
import numpy as np
import subprocess,re
from PIL import Image

O=Path(__file__).resolve().parent;R=O.parents[1]
def read(p):
    op=gzip.open if str(p).endswith('.gz') else open
    with op(p,'rt') as f:return list(csv.DictReader(f))
def write(p,rows):
    op=gzip.open if str(p).endswith('.gz') else open
    with op(p,'wt',newline='') as f:
        w=csv.DictWriter(f,fieldnames=rows[0].keys());w.writeheader();w.writerows(rows)
def sha(p):
    h=hashlib.sha256()
    with p.open('rb') as f:
        for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
    return h.hexdigest()
a=read(O/'tables/per_attempt.csv');sc=read(O/'tables/per_scenario.csv');cy=read(O/'signals/upper_cycles.csv.gz')
# Normalize two schema labels if this finalize follows a pre-correction extraction.
p=O/'signals/B_state.csv.gz'
b=read(p);keys=['stamp_ros_s','B_before_m','B_after_m','threshold_m','candidate_length_m','candidate_step','candidate_interval']
write(p,[dict(zip(keys,x.values())) for x in b])
rt=read(O/'tables/runtime.csv')
for row in rt:
    if row['measurement']!='upper_accepted_cycles_body_plus_report_ms':continue
    n=int(row['scenario']);aa=[x for x in a if x['route']==f'scenario{n}']
    vals=[float(c['body_plus_own_report_ms']) for c in cy if c['outcome']=='success' and c['body_plus_own_report_ms'] and any(float(x['start_epoch_s'])<=float(c['t_start'])<float(x['end_epoch_s']) for x in aa)]
    row.update(n=len(vals),median=float(np.median(vals)),p95=float(np.quantile(vals,.95)),p99=float(np.quantile(vals,.99)),maximum=max(vals))
write(O/'tables/runtime.csv',rt)
assert len(a)==60 and all(x['departed']=='True' for x in a)
assert all(x['stop_status']=='stationary' for x in a)
assert all(float(x['first_B_before_m'])==0 and x['first_waypoint_pair']=='[1, 2]' for x in a)
assert sum(int(s['completed']) for s in sc)==25
pages={}
for p in (O/'figures').glob('*.pdf'):
    info=subprocess.check_output(['pdfinfo',str(p)],text=True)
    pages[p.name]=int(re.search(r'Pages:\s+(\d+)',info)[1])
assert pages['scenario_all_attempts.pdf']==12
assert pages['state_error_command_histories.pdf']==12
for p in (O/'figures').glob('*.png'):
    with Image.open(p) as im:im.verify()
assert len(list((O/'figures').glob('scenario*_all5.png')))==12
upper_fail=sum(int(s['upper_failed']) for s in sc);upper_n=sum(int(s['upper_attempts']) for s in sc)
trackn=sum(int(x['tracking_samples']) for x in a);uppermax=max(float(s['upper_runtime_max_ms']) for s in sc)
start=min(float(x['start_epoch_s']) for x in a);stop=max(float(x['stop_epoch_s']) for x in a)
lines=['2026-10-09 전체12개 시나리오 각5회 빠른 결과 분석',
'run ID: 20261009_070714_all_d8c3cf',
'원본: /home/imac/ros2_ws/drive_batches/20261009_070714_all_d8c3cf',
'분석 범위: 결과 보고·전 회차 시각화·추종·시간·실패 기록. 원인 분리 재실험/튜닝 없음.',
'', '핵심 결과',
'- 예정60회 모두 실제 running 진입. 목표 영역 접근25회, off_route33회, stopped_10s2회.',
'- 시스템 이상 생략0회, 모든 회차 stationary 정지 확인, 최종 phase=done.',
'- done은 일정 종료를 뜻하며 모든 시행 성공을 의미하지 않는다.',
'- 매 회 첫 B_before=0, 첫 waypoint=[1,2]. 이전 임무 초기화 누락 근거 없음.',
f'- 첫 running부터 마지막 정지까지 {(stop-start)/60:.2f}분. 준비/관성 대기 포함.',
'', '시나리오별 (접근/이탈/정체, 전체5회씩)']
for s in sc:
    n=int(s['scenario']);aa=[x for x in a if x['route']==f'scenario{n}']
    outcomes=' / '.join(f"A{x['attempt']}:{x['outcome']}" for x in aa)
    lines += [f"S{n}: {s['completed']}/{s['off_route']}/{s['stopped_10s']} | {outcomes}",
              f"  lower-path 횡오차 RMSE={float(s['lateral_rmse_m']):.3f}m, command포화={float(s['command_saturation_pct']):.2f}%, upper미발행={s['upper_failed']}/{s['upper_attempts']}"]
lines += ['', '궤적에서 확인한 결과 (도로 내 주행·충돌 판정과 구분)',
'- S1: A1은 마지막 분기에서 북쪽으로 진행. A2~5는 목표 영역 접근.',
'- S2: A1~2는 경로 후반에서 stopped_10s. A4는 중간 분기에서 남쪽 진행.',
'  A3/A5는 후반 경로 이탈. 정체의 계산·입력 원인은 이 빠른 분석에서 확정하지 않음.',
'- S3: 이번5회 모두 목표 접근 실패. A1/A2/A5는 남동쪽 진행, A3는 동쪽 진행,',
'  A4는 북동쪽으로 방향을 바꾸지만 목표 접근 전 off_route. 모두 같은 오분기로 단정하지 않음.',
'- S4/S5/S9: 각5회 목표 영역 접근. S5/S9는 명령 포화0%, 횡오차 RMSE 약0.072/0.052m.',
'- S6: A1~4 접근, A5는 첫 주요 방향 전환에서 남쪽 대신 동쪽 진행.',
'- S7/S8: 모두 경로 이탈. S7은 주로 북쪽 방향, S8은 동쪽 출구 대신 북쪽 진행이 반복됨.',
'- S10: A2만 접근. 나머지는 앞부분 분기에서 북동쪽 대신 동쪽 진행.',
'- S11: A1만 접근. A2~5는 마지막 방향 전환에서 서쪽 대신 남쪽 진행.',
'- S12: 모두 경로 이탈. 동쪽 진행 뒤 목표 방향으로 충분히 북동 선회하지 못함.',
'위 방향 서술은 기록 궤적/waypoint 그림의 기하학적 해석이며 후보 ID 기반 자동 분기 인증이 아님.',
'', '이전 S1~3 기록과 비교',
'- 이전 접근: S1 9/10, S2 0/10, S3 10/10. 이번: S1 4/5, S2 0/5, S3 0/5.',
'- 이번 전달된 effective_config의 EKF/upper/lower 제어 파라미터는 이전 기록과 동일.',
'  px4_odom_map_bridge.route_name 초기값만 전체 세션 첫 경로 S1로 표시됨.',
'  실제 S2/S3 출발은 경로별 재정렬하며 최초 waypoint/B 초기화는 기록으로 확인.',
'- 비교는 실행 시 전달 설정 기준. 아직 환경/상태 편차를 분리하지 않았으므로 S3 악화 원인을',
'  B·초기화·특정 튜닝으로 단정하지 않는다. 단일 배치로 안정성 순위 일반화 불가.',
'- 현재 tracking_control_split.yaml은 manifest에 기록된 당시 파일과 SHA256이 다르다.',
'  현재 파일을 실행 당시 원본으로 복사하지 않았다. 실제 전달 effective_config를 보관함.',
'', '추종 지표/시간 정의',
f'- running 중 유효 QP 또는 오차 피드백이 있는 mode0/1/2/5 표본 {trackn}개로 추종오차 집계.',
'  비활성/중립 표본의 placeholder 0을 추종오차에 포함하지 않았다.',
'- 횡오차/heading 오차는 하위 참조 경로 기준이며 독립 도로 중심선 오차가 아니다.',
'- 조향은 trace[26] 출력 명령(rad-equivalent). 실제 바퀴각 측정이 아니다.',
'  포화는 abs(command)>=0.3-1e-5rad. 명령 변화율 초과 쌍도 시행표에 별도 수록.',
'  변화율은 연속 실제 출력 차분/기록 시각 차분이며 QP 내부 rate 제약 판정과 다름.',
f'- running 상위 미발행 {upper_fail}/{upper_n}회. 전부를 단일 solver 실패라고 해석하지 않는다.',
f'- 상위 전체 cycle body + own report 관측 최대 {uppermax:.3f}ms. 500ms 초과0회.',
'  모든 실패를 포함한 cycle 통계와 success cycle 통계를 runtime.csv에 별도 수록.',
'  이전 cycle의 previous_report_ms를 다음 cycle에서 읽어 해당 cycle 자체 보고비용을 더함.',
'  마지막 연속 cycle 보고가 없는 표본은 합산 시간 통계에서 제외. callback dispatch 대기는 제외.',
'- 하위 solver 실제 호출 표본 중 invalid QP0회. 미호출 중립 표본과 구분.',
'  하위 trace[24] solver 시간 및 trace[11] 구성~solver반환 시간은 전체 callback 시간이 아니다.',
'  따라서 전체 하위 deadline miss는 평가 불가. 관측 최대값은 WCET 보장이 아니다.',
'- 시간 P50/P95/P99/최대/표본수는 tables/runtime.csv. 단위ms, 시계측정 지점은 기존 소스 참조.',
'', '그림 및 출처',
'- figures/all_60_routes.png/pdf: 모든 시나리오 전체 경로. 각 시나리오 5개 궤적.',
'- figures/scenario_all_attempts.pdf: 12페이지, 각 페이지 시나리오 전체+개별5회 (총60회).',
'- figures/scenarioN_all5.png: 시나리오별 고해상도 그림.',
'- figures/state_error_command_histories.pdf: 12페이지, 전 회차 속도/횡오차/heading오차/출력 명령.',
'- figures/results_metrics.png/pdf, runtime.png/pdf: 결과/추종/실행시간 집계.',
'- 검은 점선은 실행 당시 routes/ CSV, 실선은 running 궤적, 색 점선은 관성 이동.',
'  x는 결과 판정 직전 마지막 기록 위치, 작은 사각형은 실제 정지 위치. 별은 목표.',
'- 차량 위치는 /motive/vehicle/odom_map, map 좌표m, orientation.z scalar yaw rad.',
'  이 구성은 PX4 EKF에서 얻은 가상 차량 상태이며 물리 차량 측정/독립 SIH truth와 구분.',
'- 배경은 route_design/road_mask.npy의 0.5m 표시 사본(configs/road_backdrop.npz).',
'  processed road map 재구성 그림이며 실제 운용 grid와 독립 도로 안전 ground truth로 사용하지 않음.',
'- 모든 그림에 run ID, 개별 회차 그림에 KST 시각. per_attempt.csv에 epoch/KST/회차 연결.',
'  phases.json 및 signals/*.csv.gz로 재계산 가능. 원시 signal과 사후 파생 지표 구분.',
'- upper 실패 사건 시각/위치는 signals/upper_events.csv.gz, 이유별 횟수는 per_attempt.csv.',
'- sources_sha256.json은 원본 모든 파일 hash; execution_sources/hash_verification.json은',
'  manifest와 현재 소스/실행파일 대조. 22개 중21개 일치. 비일치 기본 YAML은 당시 원본으로 보관하지 않음.',
'- occupancy grid 76,773개는 원본 MCAP에 보존. 이 분석에서는 디코딩하지 않았으며 지도 캐시 없음.',
'', '판정과 한계',
'- completed는 진행률>=85%, 남은 경로<=12m, 목표거리<=8m의 목표 접근 판정.',
'  목표 정차 성공이 아님. 실제 정지 위치/목표 거리는 시행표에 별도 기재.',
'- off_route는 waypoint 경로와 거리>15m가3초 지속된 판정. 도로 경계 이탈·충돌과 동일하지 않음.',
'- stopped_10s는 0.5m 이동 기준 갱신이10초 없었던 판정. 원인 명칭이 아님.',
'- 전체 차량 외곽 충돌/최소 경계 여유/계획 node 제약/편향 조건 검증은 이번 빠른 분석에서 미평가.',
'- 기록 상태/실패 궤적을 보여주는 자료이며 향후 폐루프 성공이나 원인을 입증하지 않는다.',
'- 앞선 07:00:42/07:02:17 시작 실패 두 세션은 별도 원본으로 보관하고60회 통계에서 제외.',
'  07:02:17의 heading guard 차단 기록은 이전 진단 참고. 이후 정상 출발60회라는 사실과 구분.',
'', '실행/복원',
'이 분석은 파일 읽기/그림 생성만 수행. 제어·맵·튜닝 변경, 주행, ARM 실행 없음.',
'분석 스크립트: analyze.py → finalize.py (ROS Humble/workspace source 후, DOMAIN177/localhost).',
'새 산출물만 생성했으므로 운영 코드 원복 불필요. 원본 로그 삭제하지 않음.',
'원본 압축: followup_20261009/raw_HIL_all60_20261009.tar.gz (최신60회+별도 시작실패2세션).',
'Git 업로드 대상: logs/archive-20261006, 기존 자료 보존 후 추가.']
(O/'RESULT.txt').write_text('\n'.join(lines)+'\n')
links=['RESULT.txt','summary.json','tables/per_attempt.csv','tables/per_scenario.csv','tables/runtime.csv']+[str(p.relative_to(O)) for p in sorted((O/'figures').iterdir())]
(O/'INDEX.html').write_text('<meta charset="utf-8"><title>HIL all60 review</title><h1>20261009_070714_all_d8c3cf</h1><p>60 runs: goal approach25, off_route33, stopped2. All scenarios shown.</p><ul>'+''.join(f'<li><a href="{html.escape(p)}">{html.escape(p)}</a></li>' for p in links)+'</ul>')
(O/'validation.json').write_text(json.dumps(dict(attempts=60,pages=pages,all_first_B_zero=True,all_first_wp_1_2=True,all_stationary=True,all_png_readable=True),indent=2)+'\n')
raw=R/'followup_20261009/raw_HIL_all60_20261009.tar.gz'
with tarfile.open(raw,'r:gz') as t:
    total=0
    for m in t:
        if m.isfile():
            f=t.extractfile(m);h=hashlib.sha256()
            for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
            assert h.hexdigest()==sha(R/m.name),m.name;total+=1
print('Validated raw archive members:',total,'; PDFs:',pages)
