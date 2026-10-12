"""Read existing extracted HIL signals; produce presentation figures only."""
from pathlib import Path
import os, csv, gzip, json, hashlib, subprocess
os.environ['MPLCONFIGDIR'] = '/tmp/all60_report_mpl'
os.environ['ROS_DOMAIN_ID'] = '177'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib.font_manager import FontProperties

O = Path(__file__).resolve().parent
R = O.parents[1]
S = O.parent / 'batch_070714_review'
B = R / 'drive_batches/20261009_070714_all_d8c3cf'
RUN = B.name
FONT = FontProperties(fname='/usr/share/fonts/truetype/nanum/NanumGothic.ttf')
def read(p):
    with (gzip.open(p, 'rt') if p.suffix == '.gz' else p.open()) as f:
        return list(csv.DictReader(f))
def sha(p):
    h = hashlib.sha256()
    with p.open('rb') as f:
        for block in iter(lambda: f.read(1024*1024), b''): h.update(block)
    return h.hexdigest()
def tracked_hashes():
    names = subprocess.check_output(['git','ls-files','-z'],cwd=R).decode().split('\0')
    return {n:sha(R/n) if (R/n).is_file() else None for n in names if n}

before = tracked_hashes()
attempts = read(S/'tables/per_attempt.csv')
raw = read(next((B/'results').glob('*/summary.csv')))
assert len(attempts) == len(raw) == 60
for a, b in zip(attempts, raw):
    for key in ('token','route','outcome','stop_status','stop_goal_distance_m'):
        assert a[key] == b[key], (key,a['token'])
assert all(a['departed']=='True' and a['stop_status']=='stationary' for a in attempts)
for p, expected in json.loads((S/'sources_sha256.json').read_text()).items():
    assert sha(R/p) == expected, p
scenarios = read(S/'tables/per_scenario.csv')
od = read(S/'signals/odom.csv.gz')
od = np.array([[float(x[k]) for k in ('bag_receive_epoch_s','yaw_scalar_rad')] for x in od])
lo = read(S/'signals/lower_trace.csv.gz')
lo = np.array([[float(x[k]) for k in ('stamp_ros_s','body_heading_error_rad','qp_valid','mode','error_feedback_available')] for x in lo])
metrics = []
colors = ['#0072B2','#D55E00','#009E73','#CC79A7','#E69F00']
with PdfPages(O/'heading_all60.pdf') as pdf:
    for n in range(1,13):
        fig, axes = plt.subplots(2,1,figsize=(11.7,8.3),sharex=True)
        for a in [a for a in attempts if a['route']==f'scenario{n}']:
            start, end = float(a['start_epoch_s']),float(a['end_epoch_s'])
            y = od[(od[:,0]>=start)&(od[:,0]<end)]
            l = lo[(lo[:,0]>=start)&(lo[:,0]<end)]
            valid = ((l[:,2]>0)|(l[:,4]>0)) & np.isin(l[:,3],[0,1,2,5]) & np.isfinite(l[:,1])
            err = np.where(valid,np.degrees(l[:,1]),np.nan)
            i = int(a['attempt']); color = colors[i-1]
            axes[0].plot(y[:,0]-start,np.degrees(np.unwrap(y[:,1])),color=color,lw=1,label=f'A{i}: {a["outcome"]}')
            axes[1].plot(l[:,0]-start,err,color=color,lw=1)
            vals = err[valid]
            rmse = float(np.sqrt(np.mean(vals**2)))
            assert len(vals)==int(a['tracking_samples'])
            assert abs(rmse-float(a['heading_rmse_deg']))<1e-9
            metrics.append(dict(run_id=RUN,original_scenario=n,attempt=i,token=a['token'],start_kst=a['start_kst'],end_kst=a['end_kst'],outcome=a['outcome'],valid_samples=len(vals),heading_rmse_deg=rmse,heading_abs_p95_deg=float(np.quantile(abs(vals),.95)),heading_abs_max_deg=float(max(abs(vals)))))
        axes[0].set_ylabel('map vehicle yaw [deg, unwrapped]')
        axes[1].set_ylabel('body heading error [deg]')
        axes[1].set_xlabel('Time since each running start [s]')
        axes[1].axhline(0,color='black',lw=.6)
        for ax in axes: ax.grid(alpha=.25)
        axes[0].legend(fontsize=8,ncol=3)
        fig.suptitle(f'Original scenario {n} | all 5 attempts\n{RUN}',fontsize=12)
        fig.text(.04,.025,'Top: scalar odom orientation.z; unwrap separately per attempt. Bottom: lower reference-path error.\nError shown only with valid QP or error feedback in modes 0/1/2/5; gaps are excluded samples. Not EKF-SIH difference.',fontsize=8)
        fig.tight_layout(rect=(0,.065,1,.94))
        pdf.savefig(fig)
        fig.savefig(O/f'heading_scenario{n}.png',dpi=150)
        plt.close(fig)
with (O/'heading_per_attempt.csv').open('w',newline='') as f:
    w=csv.DictWriter(f,fieldnames=metrics[0].keys());w.writeheader();w.writerows(metrics)

fig, ax = plt.subplots(figsize=(11.7,8.3));ax.axis('off')
ax.text(.02,.96,'전체 HIL 60회 결과·궤적·헤딩 보고',fontproperties=FONT,fontsize=21,va='top')
ax.text(.02,.89,f'2026-10-09 | {RUN}\n원래 시나리오 1~12 × 각 5회 · 선택 장면이 아닌 전체 시행',fontproperties=FONT,fontsize=11,va='top')
table = [[s['scenario'],s['completed'],s['off_route'],s['stopped_10s'],s['stationary_stops']] for s in scenarios]
t=ax.table(cellText=table,colLabels=['Original S','Goal approach','off_route','stopped_10s','Stop confirmed'],bbox=[.02,.29,.96,.50],cellLoc='center');t.auto_set_font_size(False);t.set_fontsize(10)
ax.text(.02,.24,'목표 접근 25/60 (41.7%) · off_route 33/60 (55.0%) · stopped_10s 2/60 (3.3%)\n60회 모두 출발·정지 확인, 시스템 생략 0회. completed는 목표 정차 성공이 아닙니다.\n궤적: running 실선 / 관성 이동 점선. 배경은 설계 지도 재구성입니다.\n헤딩: map yaw와 하위 경로 오차를 구분하며 EKF–SIH 차이를 나타내지 않습니다.\n원인 분석·튜닝 없음. 보고 번호 1~7의 선택 자료와 구분하여 읽어 주세요.',fontproperties=FONT,fontsize=11,va='top',linespacing=1.7)
fig.savefig(O/'summary.png',dpi=150);fig.savefig(O/'summary.pdf');plt.close(fig)
subprocess.run(['pdfunite',str(O/'summary.pdf'),str(S/'figures/all_60_routes.pdf'),str(S/'figures/scenario_all_attempts.pdf'),str(O/'heading_all60.pdf'),str(O/'REPORT_ALL60.pdf')],check=True)
assert before == tracked_hashes()
report = '''전체 HIL 60회 보고 자료 (원래 시나리오 번호 1~12)
대상: 20261009_070714_all_d8c3cf, 2026-10-09 기록. 보고 정리일 2026-10-12 KST.
REPORT_ALL60.pdf: 26페이지 (요약1 + 전체 궤적1 + 시나리오별 궤적12 + 헤딩12).
heading_all60.pdf: 모든 60회 map yaw / 하위 참조 경로 기준 body heading error.
heading_per_attempt.csv: 회차별 유효 표본 수, RMSE, 절대오차 P95/최대, run/token/KST.

결과: 목표 영역 접근25회, off_route33회, stopped_10s2회. 출발60, 정지 확인60, 시스템 생략0.
목표 접근 비율25/60=41.7%는 이 배치의 관측 비율이며 일반화된 성공률이 아님.
completed는 목표 접근 판정이며 목표 정차 성공이 아님. 정지 확인은 목표 정차와 별개.
off_route는 waypoint 경로 이탈 판정이며 충돌/도로 경계 이탈 판정이 아님.
stopped_10s는 이동 갱신 정체 판정이며 원인 명칭이 아님.

지도 yaw는 odom orientation.z scalar yaw이며 회차별 unwrap 후 degree 표시.
body heading error는 하위 참조 경로 기준. 유효 QP 또는 오차 피드백 mode0/1/2/5만 표시.
비활성 구간은 빈 구간으로 남기며 placeholder 0을 오차 통계에 포함하지 않음.
회차별 오차 RMSE/표본 수는 기존 per_attempt.csv와 대조 검증.
SD-map yaw bias 및 EKF–SIH heading 차이와 별개 신호. 이번 그림에서 해당 차이를 평가하지 않음.
기존 조향 trace는 출력 명령이며 실제 바퀴각 측정이 아님.
궤적 PDF는 기존 분석의 보존본을 그대로 병합. 각 회차 시각과 결과는 기존 표 참조.
보고 번호1~7의 선택 장면 자료는 별도 자료이며 이 60회 집계와 혼합하지 않음.
0.9rad 폐기 대상 및 +0.17/+0.09rad 별도 배치는 이 전체60회 통계에 포함하지 않음.

읽기 전용 원본 hash 검증 및 원본 summary.csv와 60회 결과/정지 대조 수행.
새 파일만 생성. 운영 코드/설정, 기존 미커밋 변경 보존. ROS 노드/주행/ARM/설정 변경 없음.
기존 추출 CSV만 읽었으며 occupancy grid 디코딩 및 캐시 생성 없음.
원격 업로드: 2026-10-12 조회에서 logs/archive-20261006의 HEAD=beab5c05162b64bea2b002966d7e66f78776d595 확인. 보고용1~7 업로드 완료. 신규 전체60회 정리 자료는 로컬 생성만 수행.
'''
(O/'REPORT_KO.txt').write_text(report)
(O/'validation.json').write_text(json.dumps(dict(attempts=60,source_hashes_verified=True,raw_results_matched=True,heading_samples_rmse_matched=True,tracked_files_unchanged=True,remote_push_status='verified: remote branch HEAD beab5c05162b64bea2b002966d7e66f78776d595'),indent=2)+'\n')
(O/'provenance_sha256.json').write_text(json.dumps({str(p.relative_to(R)):sha(p) for p in [S/'signals/odom.csv.gz',S/'signals/lower_trace.csv.gz',S/'tables/per_attempt.csv',S/'tables/per_scenario.csv',S/'figures/scenario_all_attempts.pdf',S/'figures/all_60_routes.pdf',next((B/'results').glob('*/summary.csv'))]},indent=2)+'\n')
print('Created REPORT_ALL60.pdf and 12 heading figures; verified all 60 attempts and tracked files.')
