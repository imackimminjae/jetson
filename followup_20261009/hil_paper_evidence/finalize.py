#!/usr/bin/env python3
"""Build delivery index, concise PDF tables, provenance, integrity report, and archives."""
from pathlib import Path
from collections import Counter
import csv,gzip,hashlib,html,json,os,re,subprocess,tarfile
os.environ.setdefault('MPLCONFIGDIR','/tmp/hil_paper_matplotlib')
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib.font_manager import FontProperties
OUT=Path(__file__).resolve().parent;ROOT=OUT.parents[1]
FONT=FontProperties(fname='/usr/share/fonts/truetype/nanum/NanumGothic.ttf')
plt.rcParams.update({'pdf.fonttype':42,'font.family':FONT.get_name(),'font.size':10})
def readcsv(p):
    op=gzip.open if str(p).endswith('.gz') else open
    with op(p,'rt') as f:return list(csv.DictReader(f))
def writecsv(p,rows):
    keys=list(dict.fromkeys(k for r in rows for k in r))
    with p.open('w',newline='') as f:w=csv.DictWriter(f,fieldnames=keys);w.writeheader();w.writerows(rows)
def dump(p,j):p.write_text(json.dumps(j,indent=2,ensure_ascii=False)+'\n')
def sha(p):
    h=hashlib.sha256()
    with p.open('rb') as f:
        for b in iter(lambda:f.read(1048576),b''):h.update(b)
    return h.hexdigest()

missing=[
 ('추가 분기 시점/다른 회차 상세 장면','기존 로그로 복원 가능','원 MCAP, upper events, lower path, odom','추가 장면 선정; 격자-차량 source 시각은 여전히 불명'),
 ('현재 실행의 전체 화면·주요 하드웨어 사진','추가 계측 필요','해당 run ID에 대응하는 파일 없음','실험 화면과 실물 장치·배선 촬영; 촬영시각/run ID 기록'),
 ('Isaac host/OS/version/USD/맵 축척','추가 계측 필요','외부 simulator 실행 snapshot 없음','버전·host 사양·USD hash/metersPerUnit 및 실제 pose 적용 graph 보존'),
 ('실제 BEV 분할 문턱·카메라 calibration·TF','추가 계측 필요','제공 소스 기본값만 있고 per-run 생성기 설정 없음','외부 YAML/dump/hash/CameraInfo/TF 및 ground plane 기록'),
 ('원 영상→grid→계획 동기/지연','추가 계측 필요','모든 grid header stamp=0; bev_source_timing 미수신','source stamps + input sequence + 같은 시계 동기 상태 기록'),
 ('lower 전체 callback 실행시간/deadline','추가 계측 필요','solver 및 build→solver return 부분만 기록','모든 조기 반환 포함 callback entry/exit 및 예정 release time'),
 ('upper 지연 760ms 발생 지점의 최종 원인','추가 계측 필요','interval stage wall/CPU 차이는 있으나 내부 대기 위치 불명','stage 내부 이벤트·로그/락 대기·스케줄링 trace'),
 ('실제 주행 시 CPU/GPU 부하/전력모드/열','추가 계측 필요','현재 호스트 식별만 가능','해당 run 중 부하/클럭/온도/전력 설정과 시간 기록'),
 ('주행별 PX4 firmware/전체 SIH 상수','추가 계측 필요','과거 board identity/status만 존재','해당 실행 firmware hash·parameter dump·SIH model source hash'),
 ('실제 조향각','추가 계측 필요','명령값과 내부 모델 추정치뿐','SIH 내부 steering state 기록 또는 실물 wheel sensor; 둘 구분'),
 ('차량 중심의 실제 도로 이탈','추가 계측 필요','독립 도로 polygon과 동기 truth 상태 없음','동기 SIH truth + independent road geometry'),
 ('차량 외곽 이탈/충돌/최소 경계 여유','추가 계측 필요','폭/collision mesh/후방 도로/접촉 flag 없음','실제 가상차량 형상+road polygon+truth pose+collision flags'),
 ('독립 도로 중심선 오차','추가 계측 필요','SD waypoint를 도로 중심선으로 대체할 수 없음','도로 제작 원본의 centerline과 좌표 변환 보존'),
 ('SD-map 위치/heading 편향 HIL 결과','추가 실험 필요','확인한8배치 모두 bias disabled','기존 튜닝 고정, 변환 경로와 nominal 경로/truth 동시 기록, 조건별 반복'),
 ('목표 정차 성공률','추가 실험 필요','completed는 접근; 실제 stop은 멀리 떨어짐','정차 기준·제동 전달 계측을 갖춘 별도 시험'),
 ('통제된 입력 이상 복구 성공률','추가 실험 필요','자연 발생 실패 사례만 있음','격리된 HIL 장애 조건·지속시간·복구 조건 사전 정의'),
 ('WCET/모든 deadline 만족 보장','현 자료로 주장 불가','관측 표본 최대뿐; 실제 1회 upper 초과','추가 시험만으로 이론적 WCET 보장이 생기지는 않음')]
writecsv(OUT/'tables/missing_data.csv',[dict(item=a,status=b,current_evidence=c,next_needed=d) for a,b,c,d in missing])
print('Missing-data table ready',flush=True)

# Receive rates are not the inverse median interval and are not sensor acquisition rates.
rates=[]
for s in (1,2,3):
    trials=[r for r in readcsv(OUT/'tables/trials.csv') if int(r['scenario'])==s]
    for name,path,timecol in [('grid',OUT/f'signals/s{s}_grid_times.csv.gz','receive_s'),('odometry',OUT/f'signals/s{s}_odometry.csv.gz','receive_s')]:
        times=np.array([float(r[timecol]) for r in readcsv(path)])
        for scope in ('entire_bag','running'):
            arrays=[times] if scope=='entire_bag' else [times[(times>=float(r['running_start_ros_s']))&(times<float(r['running_end_ros_s']))] for r in trials]
            diffs=np.concatenate([np.diff(t) for t in arrays if len(t)>1]);exposure=sum(t[-1]-t[0] for t in arrays if len(t)>1)
            rates.append(dict(scenario=s,signal=name,scope=scope,samples=sum(len(t) for t in arrays),interval_samples=len(diffs),receive_rate_hz=len(diffs)/exposure,median_interval_s=float(np.median(diffs)),p95_interval_s=float(np.quantile(diffs,.95)),p99_interval_s=float(np.quantile(diffs,.99)),max_interval_s=float(max(diffs)),clock='bag receipt time; not source acquisition'))
writecsv(OUT/'tables/grid_and_state_rates.csv',rates)
print('Receive-rate table ready',flush=True)

# Explicit numeric branch labels where supported by trajectory; unknown road containment remains separate.
branch=[]
for r in readcsv(OUT/'tables/trials.csv'):
    s,a=int(r['scenario']),int(r['attempt']);out=r['runner_outcome']
    choice='intended exit sequence observed' if out=='completed' else 'not fully traversed; no final intended-exit success'
    if s==1 and a==9:choice='north instead of intended east at final branch'
    if s==2 and a in (7,10):choice='south at middle branch instead of continuing east'
    if s==2 and a==3:choice='late southeast departure after neutral/control transitions; endpoint failure'
    branch.append(dict(scenario=s,attempt=a,run_id=r['run_id'],runner_outcome=out,exit_observation=choice,evidence='recorded map trajectory + waypoint route; human geometric annotation, not persistent candidate ID',road_center_containment='not independently assessable',full_footprint_containment='not assessable'))
writecsv(OUT/'tables/exit_and_road_results.csv',branch)
print('Exit annotations ready',flush=True)

def table_page(title,cols,rows,notes,colWidths=None):
    fig,ax=plt.subplots(figsize=(11.69,8.27));ax.axis('off');ax.set_title(title,fontproperties=FONT,fontsize=17,pad=24)
    tab=ax.table(cellText=rows,colLabels=cols,loc='upper center',bbox=[0,.36,1,.6],cellLoc='center',colWidths=colWidths);tab.auto_set_font_size(False);tab.set_fontsize(9)
    for (row,col),cell in tab.get_celld().items():
        cell.set_text_props(fontproperties=FONT)
        cell.set_edgecolor('#d4dbe2');cell.set_facecolor('#dce8f2' if row==0 else ('#f5f8fa' if row%2 else 'white'))
    ax.text(0,.29,notes,transform=ax.transAxes,fontproperties=FONT,fontsize=10,va='top',linespacing=1.5)
    fig.text(.5,.02,'2026-10-09 | HIL evidence package | 원본·범위·계산법: REPORT_KO.txt 및 sources/',ha='center',fontproperties=FONT,fontsize=9);fig.subplots_adjust(left=.045,right=.955,top=.89,bottom=.06)
    return fig
summ=readcsv(OUT/'tables/scenario_summary.csv');runtime=readcsv(OUT/'tables/runtime_statistics.csv')
with PdfPages(OUT/'figures/00_evidence_tables.pdf') as pdf:
    rows=[[r['scenario'],r['trials'],r['goal_approach'],r['off_route'],r['stopped_10s'],f"{float(r['lateral_rmse_m']):.3f}",f"{r['upper_failures']}/{r['upper_attempts']}"] for r in summ]
    fig=table_page('기존 HIL 30회 검증 결과', ['시나리오','시행','목표 접근','경로 이탈','정체','횡오차 RMSE(m)','상위 실패/시도'],rows,
     '세 시나리오 각각 최신 10회 전체를 포함. 성공 회차만 선별하지 않음.\ncompleted는 목표 영역 접근이며 목표 정차 성공이 아님.\n횡오차는 현재 하위 추종 경로 기준이며 독립 도로 중심선 오차가 아님.\nS1 목표 접근 9회, S3 10회의 실제 최종 정지점은 목표에서 약 32~37m 떨어짐.\nS2의 작은 추종오차가 출구 선택 또는 완주 성공을 뜻하지 않음.');pdf.savefig(fig);plt.close(fig);print('PDF table page1 ready',flush=True)
    rows=[]
    for r in runtime:
        if r['scope']=='running' and r['metric'] in ('upper_cycle_including_report','lower_solver_all_called'):
            label='상위 측정 사이클' if r['metric'].startswith('upper') else '하위 솔버만'
            rows.append([r['scenario'],label,r['n']]+[f"{float(r[k]):.3f}" for k in ('median','p95','p99','maximum')]+[r['above_period_count'] if label.startswith('상위') else '전체 미계측'])
    fig=table_page('실행시간: 계측 범위를 구분한 통계', ['시나리오','계측 범위','n','중앙값 ms','P95 ms','P99 ms','최대 ms','주기 초과'],rows,
     '상위 주기 500ms: 자체 보고 비용 포함, callback 진입 전 대기는 제외. S2 1회 초과(0.0803%).\nS2 A02 18:42:06.732: 허용 구간 추출 단계 wall 758.635ms / thread CPU 3.409ms.\n그때 solver는 0.110ms. 지연의 근본 원인은 미분리.\n하위 주기 100ms 전체 callback은 미계측. 솔버 통계로 deadline miss 0회를 주장하지 않음.\n성공만/실패 포함/배치 전체의 분리 통계는 runtime_statistics.csv. 최대값은 WCET가 아님.');pdf.savefig(fig);plt.close(fig);print('PDF table page2 ready',flush=True)
    rows=[['상위','2Hz, dt .75s, 최대 N6','실행 설정 + 로그'],['하위','10Hz, dt .1s, N30','실행 설정 + trace'],['목표속도 / 휠베이스','6m/s / 2.8m','실행 설정 + 로그'],['길이 / 폭','4.7m / 미확보','parameter event / 미확보'],['격자','128×128, .3125m, x0~40/y±20m','73,250 원 프레임'],['하위 조향 / 변화율','.3rad / 1.0472rad/s 설정','실행 설정, 실제 명령 별도 검사'],['B/soft','relative factor1.5, ratio .7, margin2m','실행 설정'],['물리 / 가상','Jetson·PX4 보드 / SIH 차량·센서·Isaac 도로','구성·과거 보드 기록'],['편향 기능','8개 배치 모두 disabled','보존된 effective_config']]
    fig=table_page('실제 설정과 확인 범위', ['구성','값','증거'],rows,
     'fullscale: 제어 단위 m. Isaac stage의 실제 축척·버전·외부 BEV YAML은 미확보.\nDAQP vendor 소스 버전0.7.2. 이번 상위 후보 조합은 모두256 이하: 조합별 연속 QP 비교.\nodom.orientation.z는 scalar yaw. 위치는 EKF 유래 가상 상태이며 모션캡처 실측이 아님.\n실험 화면/하드웨어 사진·실물 바퀴각·독립 도로 경계·차량 폭·편향 HIL 시험은 미확보.\n현재 호스트/과거 보드 확인값과 해당 주행 적용값을 CONFIGURATION_KO.txt에서 구분.');pdf.savefig(fig);plt.close(fig);print('PDF table page3 ready',flush=True)

# Register supplement figures and data-table provenance.
fp=OUT/'sources/figure_provenance.csv';provenance=readcsv(fp)
extra=[('00_evidence_tables.pdf','latest30; configuration evidence','tables/scenario_summary.csv; runtime_statistics.csv; configs','display of recomputable tables'),('07_upper_deadline_overrun','S2 A02; plan254; t=1791452526.7327998','signals/s2_upper_stage_timing.jsonl.gz','wall vs CPU by stage; no root-cause attribution'),('08_historical_input_loss','20261008_173306_scenario1_756dad A05; last1791448677.6254458','historical input bag; signals/historical_s1_a05_*','break pose across gap; stale lower state annotated')]
known={r['figure'] for r in provenance}
for base,scope,signals,method in extra:
    for name in ([base] if base.endswith('.pdf') else [base+'.pdf',base+'.png']):
        if 'figures/'+name not in known:provenance.append(dict(figure='figures/'+name,scope=scope,timestamps_ros_s=scope,signals=signals,method=method))
writecsv(fp,provenance)
metricmethods=[
 ('tables/trials.csv','all latest30, running[start,end); coast separate','signals/s*_odometry.csv.gz; lower_trace; batch_status; upper_events','distance=sum consecutive XY steps; errors RMSE/max over error-valid control modes; rates use explicit nominal/elapsed definitions'),
 ('tables/scenario_summary.csv','all latest30, per scenario','tables/trials.csv + concatenated valid lower samples','sample-weighted error RMSE; runner outcome counts; no success-only filtering'),
 ('tables/runtime_statistics.csv','running and entire_bag separately; all-called and success separately','upper_stage_timing; upper_events; lower_trace','NumPy linear quantiles .5/.95/.99 and observed max; >period strictly; missing values excluded, n disclosed'),
 ('tables/plan_interval_checks.csv.gz','valid published-plan cycles in running','upper_events selected corridor endpoints, planned_world','lateral scalar projection; reject below -0.001m; guard-tail excluded; not collision/road containment'),
 ('tables/event_counts_by_scope.csv','running and entire_bag','upper_events and lower_trace modes','per-cycle/message counts, not independent episodes'),
 ('tables/output_rate_exceedances.csv','consecutive running samples in same attempt, 0<dt<.3','lower_trace command26, delta_prev29, mode12','abs(delta_command)/dt >1.0471975512+1e-4rad/s; categorize reference change vs nominal step tolerance'),
 ('tables/grid_and_state_rates.csv','running and entire_bag','grid receipt and odom receipt times','average receive rate=number intervals/sum within-phase spans; not inverse median interval'),
 ('tables/exit_and_road_results.csv','all latest30','map trajectories + route + outcomes','manual geometric exit observation separate from unavailable independent road containment'),
 ('tables/applied_settings.csv','latest 3 execution configs','effective_config + recorded parameter_events','all applied launch values; only corroborated events labeled as such')]
writecsv(OUT/'sources/metric_provenance.csv',[dict(file=f,scope=s,signals=g,method=m) for f,s,g,m in metricmethods])

selected=['00_evidence_tables.pdf','00_HIL_configuration.pdf','01_all_routes.pdf','03_s1_a06_branch_zoom.pdf','03_s2_a01_branch_zoom.pdf','03_s3_a01_branch_zoom.pdf','03_s3_a01_curve_zoom.pdf','04_s3_a01_histories.pdf','03_s1_a09_wrong_exit_zoom.pdf','03_s1_a01_failure_zoom.pdf','03_s2_a01_failure_zoom.pdf','03_s3_a04_failure_zoom.pdf','04_s2_a01_histories.pdf','05_runtime_distributions.pdf','06_failures_all_trials.pdf','07_upper_deadline_overrun.pdf','08_historical_input_loss.pdf','02_s1_all_10_attempts.pdf','02_s2_all_10_attempts.pdf','02_s3_all_10_attempts.pdf']
assert all((OUT/'figures'/name).exists() for name in selected), 'Complete make_figures.py first'
subprocess.run(['pdfunite',*[str(OUT/'figures'/p) for p in selected],str(OUT/'HIL_validation_figures.pdf')],check=True)
print('Combined PDF ready',flush=True)
writecsv(OUT/'sources/combined_pdf_contents.csv',[dict(order=i+1,source_pdf='figures/'+p) for i,p in enumerate(selected)])

figlinks='\n'.join(f'<li><a href="{html.escape(r["figure"])}">{html.escape(Path(r["figure"]).name)}</a> — {html.escape(r["scope"])}</li>' for r in provenance if r['figure'].endswith('.pdf'))
page='''<!doctype html><html lang="ko"><meta charset="utf-8"><title>HIL 검증 자료</title><style>body{font:16px/1.7 sans-serif;max-width:1080px;margin:40px auto;padding:0 24px;color:#162b40}h1{font-size:30px}a{color:#0a6299}table{border-collapse:collapse;width:100%}td,th{border-bottom:1px solid #d5dfe6;padding:10px;text-align:left}img{max-width:100%}.note{background:#edf4f8;padding:16px;border-radius:8px}small{color:#506473}</style><h1>SD-map MIQP-MPC · HIL 검증 자료</h1><p>2026-10-09 · 기존 기록 분석 · 운영 코드/설정/주행 변경 없음</p><p><a href="HIL_validation_figures.pdf">그림·설정·결과 통합 PDF</a> · <a href="REPORT_KO.txt">상세 보고서</a> · <a href="CONFIGURATION_KO.txt">HIL 구성·실제 설정</a> · <a href="BIAS_AND_MISSING_KO.txt">편향·미확보 자료</a></p><table><tr><th>시나리오</th><th>목표 접근</th><th>경로 이탈</th><th>정체</th><th>상위 실패</th></tr><tr><td>1</td><td>9/10</td><td>1</td><td>0</td><td>7/787</td></tr><tr><td>2</td><td>0/10</td><td>3</td><td>7</td><td>461/1245</td></tr><tr><td>3</td><td>10/10</td><td>0</td><td>0</td><td>6/660</td></tr></table><p class="note">completed는 목표 접근이며 정차 성공이 아닙니다. S2에는 상위 500ms 주기를 넘은 760.8ms 사례가 1회 있습니다. 하위 전체 callback 시간, 실물 조향각, 차량 외곽 충돌, HIL 편향 결과는 미확보입니다.</p><img src="figures/01_all_routes.png" alt="30회 전체 경로"><p>배경은 기록 격자를 합성한 그림이며 독립적인 도로 경계 자료가 아닙니다.</p><h2>시나리오별 전 회차</h2><p><a href="figures/02_s1_all_10_attempts.pdf">시나리오1: 10페이지</a> · <a href="figures/02_s2_all_10_attempts.pdf">시나리오2: 10페이지</a> · <a href="figures/02_s3_all_10_attempts.pdf">시나리오3: 10페이지</a></p><h2>재계산 가능한 표와 출처</h2><ul><li><a href="tables/trials.csv">시행별 결과 CSV</a></li><li><a href="tables/runtime_statistics.csv">실행시간 통계 CSV</a></li><li><a href="tables/applied_settings.csv">실제 실행 설정 CSV</a></li><li><a href="tables/failure_and_mode_events.csv">전체 실패·모드 전환 CSV</a></li><li><a href="tables/missing_data.csv">미확보 목록 CSV</a></li><li><a href="sources/signal_dictionary.csv">신호 단위·좌표계·시각·기록/재구성 구분</a></li><li><a href="sources/original_files.csv">주 분석 원본 파일·SHA256</a></li><li><a href="sources/figure_provenance.csv">그림 출처</a></li><li><a href="sources/metric_provenance.csv">지표 계산법</a></li></ul><h2>개별 논문용 그림</h2><ul>'''+figlinks+'</ul><p><small>실험 화면/하드웨어 사진은 미확보이며 기능도와 재구성 그림으로 대체해 촬영 자료처럼 표시하지 않았습니다.</small></p></html>'
(OUT/'INDEX.html').write_text(page)

# Integrity checks before packaging. Inspect every PDF and PNG; no algorithm tests are run.
from PIL import Image
validation={'pdfs':{},'pngs':{},'operating_files':{},'original_files':{},'scene_count':len(json.loads((OUT/'scenes/scene_manifest.json').read_text()))}
for p in sorted((OUT/'figures').glob('*.pdf')):
    output=subprocess.check_output(['pdfinfo',str(p)],text=True);pages=int(re.search(r'Pages:\s+(\d+)',output).group(1));validation['pdfs'][p.name]=pages;assert pages>0
for p in sorted((OUT/'figures').glob('*.png')):
    with Image.open(p) as im:validation['pngs'][p.name]=list(im.size);im.verify()
for p,value in json.loads((OUT/'sources/operating_files_before.json').read_text()).items():
    match=sha(Path(p))==value;validation['operating_files'][p]=match;assert match,p
for row in readcsv(OUT/'sources/original_files.csv'):
    match=sha(Path(row['file']))==row['sha256'];validation['original_files'][row['file']]=match;assert match,row['file']
assert all(validation['pdfs'][f'02_s{s}_all_10_attempts.pdf']==10 for s in (1,2,3))
assert len(readcsv(OUT/'tables/trials.csv'))==30
validation['combined_pdf_info']=subprocess.check_output(['pdfinfo',str(OUT/'HIL_validation_figures.pdf')],text=True)
validation['operational_changes']='none; preservation checkpoint hashes identical; no operation publishing, hardware access or driving'
dump(OUT/'sources/final_validation.json',validation)
print('Integrity checks complete',flush=True)

# Original MCAP stays compressed: archive original bytes, never decoded maps.
rawname=OUT.parent/'raw_HIL_logs.tar';rawfiles=[]
for run in sorted((ROOT/'drive_batches').iterdir()):
    if not (run/'effective_config.yaml').exists():continue
    selected_raw=list((run/'bag').glob('*'))+[run/x for x in ('effective_config.yaml','manifest.json','session_result.json','controllers.log','recorder.log')]+list((run/'results').rglob('*'))
    for p in selected_raw:
        if p.is_file():rawfiles.append(p)
writecsv(OUT/'sources/raw_archive_inventory.csv',[dict(file=str(p),archive_path=str(p.relative_to(ROOT)),bytes=p.stat().st_size,sha256=sha(p)) for p in rawfiles])
with tarfile.open(rawname,'w') as tar:
    for p in rawfiles:tar.add(p,arcname=str(p.relative_to(ROOT)),recursive=False)
with tarfile.open(rawname,'r') as tar:
    assert len(tar.getmembers())==len(rawfiles)
    for row,member in zip(readcsv(OUT/'sources/raw_archive_inventory.csv'),tar.getmembers()):
        assert row['archive_path']==member.name
        h=hashlib.sha256()
        f=tar.extractfile(member)
        for chunk in iter(lambda:f.read(1048576),b''):h.update(chunk)
        assert h.hexdigest()==row['sha256']
archiveinfo=dict(raw_archive=str(rawname),raw_files=len(rawfiles),raw_bytes=rawname.stat().st_size,raw_sha256=sha(rawname),notes='All 8 recorded batch originals; separate cohorts and setup failures retained. No decoded grid cache.')
dump(OUT/'sources/archive_info.json',archiveinfo)
artifacts=[p for p in OUT.rglob('*') if p.is_file() and p.name!='artifact_manifest.csv' and p.suffix!='.log' and '__pycache__' not in str(p)]
writecsv(OUT/'sources/artifact_manifest.csv',[dict(file=str(p.relative_to(OUT)),bytes=p.stat().st_size,sha256=sha(p)) for p in sorted(artifacts)])
bundle=OUT.parent/'hil_paper_evidence.tar.gz'
with tarfile.open(bundle,'w:gz',compresslevel=4) as tar:
    for p in sorted(OUT.rglob('*')):
        if p.is_file() and p.suffix!='.log' and '__pycache__' not in str(p):tar.add(p,arcname=str(p.relative_to(OUT.parent)),recursive=False)
with tarfile.open(bundle,'r:gz') as tar:assert len(tar.getmembers())==len(artifacts)+1
(OUT.parent/'HIL_DELIVERY_SHA256.txt').write_text(f'{sha(bundle)}  {bundle.name}\n{sha(rawname)}  {rawname.name}\n')
print(json.dumps(dict(evidence_archive=str(bundle),evidence_bytes=bundle.stat().st_size,raw_archive=str(rawname),raw_bytes=rawname.stat().st_size,pdfs=len(validation['pdfs']),pngs=len(validation['pngs']),scenes=validation['scene_count'],preserved_operating_files=len(validation['operating_files'])),indent=2),flush=True)
