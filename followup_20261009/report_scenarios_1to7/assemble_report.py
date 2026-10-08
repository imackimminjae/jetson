from pathlib import Path
import csv,gzip,json,os,shutil,tarfile
os.environ['MPLCONFIGDIR']='/tmp/success_report_mpl'
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib import font_manager
font_manager.fontManager.addfont('/usr/share/fonts/truetype/nanum/NanumGothic.ttf')
plt.rcParams.update({'font.family':'NanumGothic','axes.unicode_minus':False,'font.size':10})
r=Path('/home/imac/ros2_ws');o=r/'followup_20261009/report_scenarios_1to7';o.mkdir(exist_ok=True)
for n in ['figures','tables','configs','provenance']:(o/n).mkdir(exist_ok=True)
a=r/'followup_20261009/batch_070714_review';old=r/'followup_20261008/batches_182344_183952_185426_review';mapping=[]
with gzip.open(a/'signals/odom.csv.gz','rt') as f:d=list(csv.DictReader(f))
od=np.array([[float(x[k]) for k in ['bag_receive_epoch_s','x_map_m','y_map_m']] for x in d]);rows=list(csv.DictReader((a/'tables/per_attempt.csv').open()));mask=np.load(a/'configs/road_backdrop.npz')['road_mask'];ext=json.loads((a/'configs/road_mask.json').read_text())['ext']
with gzip.open(old/'scenario3/tracks.json.gz','rt') as f:oldtracks=json.load(f)
summary=json.loads((old/'scenario3/summary.json').read_text())
fig_all,axs=plt.subplots(2,4,figsize=(20,10))
with PdfPages(o/'REPORT_SCENARIOS_1_TO_7.pdf') as pdf:
 for reportid,sourceid in enumerate([1,2,3,4,5,6,9],1):
  run={2:'20261008_183952_scenario2_5524df',3:'20261008_185426_scenario3_72c4df'}.get(sourceid,'20261009_070714_all_d8c3cf');kind='참조 경로' if sourceid==2 else '목표 접근 주행 예시'
  path=(r/'drive_batches'/run/'routes'/f'scenario{sourceid}.csv') if sourceid not in [2,3] else r/f'src/virtual_control/data/knu_routes/scenario{sourceid}.csv'
  route=np.array([[float(x['x']),float(x['y'])] for x in csv.DictReader(path.open())]);shutil.copy2(path,o/f'configs/report{reportid}_route.csv')
  cfg=r/'drive_batches'/run/'effective_config.yaml';shutil.copy2(cfg,o/f'configs/report{reportid}_effective_config.yaml')
  paths=[];selected=[]
  if sourceid==3:
   for tr in oldtracks:
    row=summary['attempts'][tr['attempt']-1]
    if row['outcome']!='completed':continue
    t=np.asarray(tr['t']);xy=np.asarray(tr['xy']);paths.append((xy[t<tr['end']],f"A{tr['attempt']}",row['start_clock'],row['end_clock']))
    selected.append({'report_id':reportid,'source_scenario':sourceid,'run_id':run,'attempt':tr['attempt'],'start_kst':row['start_clock'],'end_kst':row['end_clock']})
  elif sourceid!=2:
   for row in rows:
    if row['route']!=f'scenario{sourceid}' or row['outcome']!='completed':continue
    xy=od[(od[:,0]>=float(row['start_epoch_s']))&(od[:,0]<float(row['end_epoch_s'])),1:3]
    paths.append((xy,'A'+row['attempt'],row['start_kst'],row['end_kst']));selected.append(dict(report_id=reportid,source_scenario=sourceid,run_id=run,attempt=row['attempt'],start_kst=row['start_kst'],end_kst=row['end_kst']))
  entry=dict(report_id=reportid,source_scenario=sourceid,run_id=run,content_type=kind,selected_examples=len(paths));mapping.append(entry)
  if selected:
   with (o/f'tables/report{reportid}_selected_examples.csv').open('w',newline='') as f:
    w=csv.DictWriter(f,fieldnames=selected[0]);w.writeheader();w.writerows(selected)
  def draw(ax,legend=False):
   ax.imshow(mask,origin='lower',extent=ext,cmap='Greys',alpha=.25,vmin=0,vmax=1)
   ax.plot(route[:,0],route[:,1],'k--o',lw=1,ms=3,label='SD-map 경로')
   for xy,label,_,__ in paths:ax.plot(xy[:,0],xy[:,1],lw=1.3,label=label)
   ax.plot(*route[0],'g^',ms=8,label='시작');ax.plot(*route[-1],'k*',ms=10,label='목표')
   pts=np.vstack([route]+[p[0] for p in paths]);lo=pts.min(0)-12;hi=pts.max(0)+12;c=(lo+hi)/2;h=max(hi-lo)/2
   ax.set_xlim(c[0]-h,c[0]+h);ax.set_ylim(c[1]-h,c[1]+h);ax.set_aspect('equal');ax.grid(alpha=.2);ax.set_xlabel('map X [m]');ax.set_ylabel('map Y [m]');ax.set_title(f'시나리오 {reportid} · {kind}')
   if legend:ax.legend(fontsize=8,ncol=3)
  draw(axs.flat[reportid-1]);fig,ax=plt.subplots(figsize=(10,9));draw(ax,True)
  times='\n'.join(f'{label}: {start} ~ {end}' for _,label,start,end in paths)
  if sourceid==2:times='기존 SD-map 참조 경로 표시 · 성공 주행 예시는 미확보'
  fig.suptitle(f'보고용 시나리오 {reportid}\n{run}',fontsize=13)
  fig.text(.06,.02,times+'\n차량 상태: PX4 EKF/map. 배경: 설계 도로 재구성. 목표 접근은 목표 정차 성공과 구분.',fontsize=7)
  fig.tight_layout(rect=(0,.15,1,.93));pdf.savefig(fig);fig.savefig(o/f'figures/scenario{reportid}.png',dpi=220);plt.close(fig)
 axs.flat[7].axis('off');fig_all.suptitle('시나리오 1~7 보고 자료 · 목표 접근 예시 모음 / 2번: 참조 경로\n선택된 장면 자료이며 전체 시행의 성능 통계가 아님',fontsize=14);fig_all.tight_layout(rect=(0,0,1,.93));fig_all.savefig(o/'figures/scenarios_1to7.png',dpi=200);plt.close(fig_all)
with (o/'provenance/scenario_number_mapping.csv').open('w',newline='') as f:
 w=csv.DictWriter(f,fieldnames=mapping[0]);w.writeheader();w.writerows(mapping)
(o/'REPORT_KO.txt').write_text('시나리오 1~7 보고 자료\n\n보고 번호: 1←기존1, 2←기존2, 3←기존3, 4←기존4, 5←기존5, 6←기존6, 7←기존9.\n1·4·5·6·7은 최신2026-10-09 기록 중 목표 접근 장면을 사용한다.\n2·3은 기존2026-10-08 자료를 사용한다. 2번은 SD-map 참조 경로 그림이며 성공 주행 예시는 미확보.\n3번은 기존 목표 접근 주행 장면을 사용한다.\n\nREPORT_SCENARIOS_1_TO_7.pdf: 7페이지 보고용 그림.\nfigures/scenarios_1to7.png: 전체 개요. figures/scenario1~7.png: 개별 그림.\n각 그림에 run ID, 주행 예시의 회차/KST 시각 표시. 원래 번호·출처는 provenance/ 대응표.\ntables/: 선택한 목표 접근 예시의 출처 시각. configs/: 해당 주행에 전달한 설정과 경로.\n\n그림 설명: 검은 점선은 SD-map waypoint, 색 실선은 목표 접근까지 실행된 궤적.\n시작점은 삼각형, 목표점은 별. 관성 정지 구간은 이 예시 그림의 주행 구간에 포함하지 않는다.\n배경은 설계 도로 자료의 재구성이며 차량 외곽 충돌/도로 내 주행 검증으로 사용하지 않는다.\ncompleted는 목표 영역 접근이며 목표 정차 성공이 아니다.\n이 자료는 성공 장면을 선택한 소개용 자료이며 전체 시행의 성공률을 제시하는 자료가 아니다.\n추가 원인 분석이나 지표 재계산 없이 기존 기록에서 장면을 선택·배치했다.\n실행 로그의 원래 시나리오 이름/번호는 바꾸지 않았다.\n\n원본 자료: 최신 raw_HIL_all60_20261009.tar.gz, 기존 raw_HIL_logs.tar (이전 업로드).\n')
(o/'INDEX.html').write_text('<meta charset="utf-8"><h1>시나리오 1~7 보고 자료</h1><a href="REPORT_SCENARIOS_1_TO_7.pdf">보고용 PDF</a> | <a href="REPORT_KO.txt">설명</a><p>목표 접근 예시 모음. 2번은 참조 경로. 전체 시행 성능 통계와 구분.</p><img width="100%" src="figures/scenarios_1to7.png">')
import hashlib
(o/'provenance/files_sha256.json').write_text(json.dumps({str(p.relative_to(o)):hashlib.sha256(p.read_bytes()).hexdigest() for p in o.rglob('*') if p.is_file() and p.name!='files_sha256.json'},indent=2)+'\n')
with tarfile.open(r/'followup_20261009/HIL_report_1to7_20261009.tar.gz','w:gz') as t:t.add(o,arcname=o.name)
print('Report 1-7 assembled; no new metrics or cause analysis.')
