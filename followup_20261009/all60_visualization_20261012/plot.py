"""Replot all original 60 trials from preserved CSV signals; no ROS or grid decode."""
from pathlib import Path
import os,csv,gzip,json,hashlib,subprocess
os.environ['MPLCONFIGDIR']='/tmp/all60_visualization_20261012_mpl'
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib.font_manager import FontProperties
from matplotlib.lines import Line2D

R=Path('/home/imac/ros2_ws');O=Path(__file__).resolve().parent
S=O.parent/'batch_070714_review';B=R/'drive_batches/20261009_070714_all_d8c3cf'
F=FontProperties(fname='/usr/share/fonts/truetype/nanum/NanumGothic.ttf')
plt.rcParams.update({'font.family':F.get_name(),'font.size':9,'axes.unicode_minus':False})
def read(p):
    with (gzip.open(p,'rt') if p.suffix=='.gz' else p.open()) as f:return list(csv.DictReader(f))
def sha(p):
    h=hashlib.sha256()
    with p.open('rb') as f:
        for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
    return h.hexdigest()
attempts=read(S/'tables/per_attempt.csv');sc=read(S/'tables/per_scenario.csv')
assert len(attempts)==60 and len(sc)==12
od=np.array([[float(x[k]) for k in ('bag_receive_epoch_s','x_map_m','y_map_m')]
             for x in read(S/'signals/odom.csv.gz')])
mask=np.load(S/'configs/road_backdrop.npz')['road_mask']
ext=json.loads((S/'configs/road_mask.json').read_text())['ext']
replay=O.parent/'route_outcome_revision/replay_summary.json'
evidence={x['token']:x for x in json.loads(replay.read_text())['per_attempt'] if x['first_confirmed_epoch_s'] is not None}
colors=['#0072B2','#D55E00','#009E73','#CC79A7','#E69F00']
tracks={};routes={};annotations=[];bounds={}
for n in range(1,13):
    routes[n]=np.array([[float(x['x']),float(x['y'])] for x in read(B/f'routes/scenario{n}.csv')])
    points=[routes[n]]
    for a in [a for a in attempts if a['route']==f'scenario{n}']:
        i=int(a['index']);start,end,stop=map(float,[a['start_epoch_s'],a['end_epoch_s'],a['stop_epoch_s']])
        tr=od[(od[:,0]>=start)&(od[:,0]<stop)];assert len(tr)>0
        tracks[i]=tr;points.extend([tr[:,1:3],np.array([[float(a['stop_x']),float(a['stop_y'])]])])
        if a['token'] in evidence:
            t=evidence[a['token']]['first_confirmed_epoch_s'];assert start<=t<end
            annotations.append(dict(token=a['token'],original_scenario=n,attempt=int(a['attempt']),
                original_outcome=a['outcome'],evidence_epoch_s=t,
                x_map_m=float(np.interp(t,tr[:,0],tr[:,1])),y_map_m=float(np.interp(t,tr[:,0],tr[:,2])),
                interpretation='Offline evidence confirmation only; not actual stop or revised outcome'))
    p=np.vstack(points);lo=p.min(0)-10;hi=p.max(0)+10;mid=(lo+hi)/2;half=max(hi-lo)/2
    bounds[n]=(mid,half)
assert len(annotations)==2

def draw(ax,n,attempt=None,legend=False):
    route=routes[n]
    ax.imshow(mask,origin='lower',extent=ext,cmap='Greys',vmin=0,vmax=1,alpha=.24)
    ax.plot(route[:,0],route[:,1],color='#252525',ls='--',lw=1.1,zorder=2)
    for a in [a for a in attempts if a['route']==f'scenario{n}' and (attempt is None or int(a['attempt'])==attempt)]:
        j=int(a['attempt']);tr=tracks[int(a['index'])];k=tr[:,0]<float(a['end_epoch_s']);c=colors[j-1]
        ax.plot(tr[k,1],tr[k,2],color=c,lw=1.35,zorder=3,label=f'A{j}: {a["outcome"]}')
        ax.plot(tr[~k,1],tr[~k,2],color=c,lw=1.05,ls=':',zorder=3)
        ax.plot(*tr[k][-1,1:3],marker='x',color=c,ms=7,mew=1.6,zorder=5)
        ax.plot(float(a['stop_x']),float(a['stop_y']),marker='s',color=c,ms=5,mfc='white',mew=1.3,zorder=5)
        ann=next((x for x in annotations if x['token']==a['token']),None)
        if ann:
            ax.plot(ann['x_map_m'],ann['y_map_m'],marker='D',color='#7A1FA2',ms=7,mfc='white',mew=1.6,zorder=6)
            if attempt is not None:
                ax.annotate('새 기준 확인\n(오프라인)',(ann['x_map_m'],ann['y_map_m']),
                            xytext=(12,12),textcoords='offset points',fontsize=8,color='#7A1FA2',
                            bbox={'boxstyle':'round,pad=.3','fc':'white','alpha':.9},
                            arrowprops={'arrowstyle':'->','color':'#7A1FA2'})
    ax.plot(*route[0],marker='^',color='#168039',ms=7,zorder=5)
    ax.plot(*route[-1],marker='*',color='black',ms=10,zorder=5)
    mid,half=bounds[n];ax.set_xlim(mid[0]-half,mid[0]+half);ax.set_ylim(mid[1]-half,mid[1]+half)
    ax.set_aspect('equal');ax.grid(alpha=.18);ax.set_xlabel('map X [m]');ax.set_ylabel('map Y [m]')
    if legend:ax.legend(fontsize=8,loc='best',framealpha=.94)

handles=[Line2D([],[],c=c,lw=2,label=f'A{i}') for i,c in enumerate(colors,1)]
handles.extend([Line2D([],[],c='black',ls='--',label='원래 waypoint'),
    Line2D([],[],c='gray',ls=':',label='관성 이동'),
    Line2D([],[],c='black',marker='x',ls='',label='원래 종료 판정 위치'),
    Line2D([],[],c='black',marker='s',mfc='white',ls='',label='실제 정지 위치'),
    Line2D([],[],c='#7A1FA2',marker='D',mfc='white',ls='',label='새 기준 확인(오프라인)')])
note='실선: running 궤적 | △ 출발, ★ waypoint 목표 | 배경: 설계 지도 재구성, 충돌·도로 경계 검증 자료 아님\ncompleted는 목표 영역 접근이며 목표 정차 성공이 아님. 보라색 ◇는 새 기준의 사후 확인 지점이며 실제 정지 위치가 아님.'
with PdfPages(O/'ALL_SCENARIOS_60_RUNS.pdf') as pdf:
    fig,axes=plt.subplots(3,4,figsize=(19,15))
    for n,ax in enumerate(axes.flat,1):
        draw(ax,n);s=sc[n-1]
        ax.set_title(f'시나리오 {n} · 전체 5회\n목표 접근 {s["completed"]} / off_route {s["off_route"]} / stopped_10s {s["stopped_10s"]}',fontsize=11)
    fig.suptitle('기존 전체 HIL 60회 궤적 · 원래 시나리오 1~12\n'+B.name,fontsize=18,y=.995)
    fig.legend(handles=handles,loc='lower center',bbox_to_anchor=(.5,.044),ncol=5,fontsize=10)
    fig.text(.03,.011,note,fontsize=9)
    fig.tight_layout(rect=(0,.105,1,.955))
    fig.subplots_adjust(hspace=.45,wspace=.32)
    fig.savefig(O/'all_scenarios_1to12.png',dpi=180);fig.savefig(O/'all_scenarios_1to12.pdf');pdf.savefig(fig);plt.close(fig)
    for n in range(1,13):
        fig,axes=plt.subplots(2,3,figsize=(15,11))
        draw(axes.flat[0],n,legend=True);axes.flat[0].set_title(f'시나리오 {n} · 전체 5회')
        for j in range(1,6):
            a=next(a for a in attempts if a['route']==f'scenario{n}' and int(a['attempt'])==j)
            draw(axes.flat[j],n,j)
            axes.flat[j].set_title(f'A{j}: {a["outcome"]}\n{a["start_kst"][11:23]} ~ {a["end_kst"][11:23]} KST',fontsize=10)
        fig.suptitle(f'원래 시나리오 {n} · 2026-10-09 기록\n{B.name}',fontsize=15,y=.99)
        fig.legend(handles=handles[5:],loc='lower center',bbox_to_anchor=(.5,.058),ncol=5,fontsize=8)
        fig.text(.035,.019,note,fontsize=8)
        fig.tight_layout(rect=(0,.1,1,.94))
        fig.subplots_adjust(hspace=.40,wspace=.28)
        fig.savefig(O/f'scenario{n}_all5.png',dpi=180);fig.savefig(O/f'scenario{n}_all5.pdf');pdf.savefig(fig);plt.close(fig)
        print(f'시나리오 {n}/12 저장 완료',flush=True)
with (O/'figure_annotations.csv').open('w',newline='') as f:
    w=csv.DictWriter(f,fieldnames=annotations[0]);w.writeheader();w.writerows(annotations)
paths=[S/'signals/odom.csv.gz',S/'tables/per_attempt.csv',S/'tables/per_scenario.csv',S/'configs/road_backdrop.npz',S/'configs/road_mask.json',replay]+[B/f'routes/scenario{n}.csv' for n in range(1,13)]
(O/'sources_sha256.json').write_text(json.dumps({str(p.relative_to(R)):sha(p) for p in paths},indent=2)+'\n')
from PIL import Image
for p in O.glob('*.png'):
    with Image.open(p) as im:im.verify()
info=subprocess.check_output(['pdfinfo',str(O/'ALL_SCENARIOS_60_RUNS.pdf')],text=True)
assert 'Pages:           13' in info
(O/'validation.json').write_text(json.dumps(dict(attempts=60,scenarios=12,pdf_pages=13,original_results_preserved=True,offline_annotations=2,all_png_readable=True,occupancy_grid_decoded=False),indent=2)+'\n')
(O/'README_KO.txt').write_text('''기존 전체 시나리오 재시각화 (2026-10-12)
원본 배치: 20261009_070714_all_d8c3cf. 원래 번호1~12, 각5회, 총60회 모두 표시.
ALL_SCENARIOS_60_RUNS.pdf: 전체 개요1 + 각 시나리오 전체/개별5회12, 총13페이지.
all_scenarios_1to12.png/pdf: 12개 시나리오 전체 개요.
scenario1~12_all5.png/pdf: 시나리오별 상세 그림.
검은 점선 waypoint, 색 실선 running, 색 점선 관성 이동, x 원래 종료 판정, □ 실제 정지.
보라색 ◇는 S2 A2/A5의 새 기준 오프라인 확인 지점. 해당 시각 위치는 odom 선형 보간.
실제 새 기준으로 주행하거나 정지한 결과가 아님. 원래 종료 결과25/33/2 그대로 표시.
completed는 목표 접근이며 목표 정차 성공이 아님. 지도 배경은 설계 자료 재구성.
참조 경로 편향 실험(+0.17/+0.09rad) 및 보고 번호1~7 선택 자료는 이60회와 별도.
헤딩 전체 자료: ../all60_report/heading_all60.pdf (12페이지).
기존 CSV와 표시용 소형 도로 배경만 읽음. occupancy grid 디코딩/캐시, ROS, 주행 없음.
기존 코드/설정/로그/보고 자료 변경 없음. 출처 hash와 표식 좌표, 검증 결과 별도 보관.
''')
