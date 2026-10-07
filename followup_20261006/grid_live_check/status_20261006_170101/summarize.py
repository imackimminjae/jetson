from pathlib import Path
import json,numpy as np,hashlib,os
os.environ['MPLCONFIGDIR']='/tmp/codex_mpl_grid'
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
p=Path(__file__).resolve().parent;h=p.parent
c=json.loads((p/'capture.json').read_text());g=c['grids']
a=json.loads((h/'publisher_params_20261006_165115.json').read_text())['values'];b=json.loads((h/'publisher_params_20261006_170105.json').read_text())['values']
diff={k:[a.get(k),b.get(k)] for k in set(a)|set(b) if a.get(k)!=b.get(k)}
summary={'first_window':json.loads((h/'20261006_165832/RESULT.json').read_text()),'second_window_frames':len(g),'free_cell_range':[min(v['free'] for v in g),max(v['free'] for v in g)],'free_bbox_patterns':sorted(set(tuple(v['free_bbox']) for v in g)),'all_frames_near_first_9m_free_zero':all(v['near_first_9m_free']==0 for v in g),'publisher_parameter_changes':diff,'upper_visible':bool(c['upper_parameters']),'events_received':c['event_count'],'limits':'No active upper planner observed. No control or configuration changes. Free bbox is not a connected usable preview range.'}
(p/'analysis.json').write_text(json.dumps(summary,indent=2));print(json.dumps({k:v for k,v in summary.items() if k!='first_window'},indent=2))
fig,axs=plt.subplots(1,2,figsize=(10,5),sharex=True,sharey=True)
for ax,folder,title in zip(axs,[h/'20261006_164936',p],['Previous capture 16:49','Current capture 17:01']):
 grid=np.load(folder/'first_grid.npy');ax.imshow(grid,origin='lower',extent=[2.95,42.95,-20,20],cmap=ListedColormap(['#308cce','#edb3b3']),vmin=0,vmax=100,interpolation='nearest');ax.set_title(title);ax.set_xlabel('Forward x (m), received origin');ax.axhline(0,color='black',ls=':',lw=.8);ax.set_xlim(0,43);ax.set_ylim(-20,20);ax.plot(0,0,'k>',clip_on=False);ax.grid(alpha=.2)
axs[0].set_ylabel('Lateral y (m)');fig.suptitle('Blue: drivable value 0 | Pink: blocked value 100\nFull 40 x 40 m coverage does not mean 40 m drivable road',fontsize=12);fig.tight_layout();fig.savefig(p/'grid_comparison.png',dpi=160);plt.close(fig)
report='''재수신 점검 (2026-10-06 16:58 및 17:01 KST, ROS domain 1)
- 최초 15초 322프레임, 약21.4Hz 수신. 40x40m, 원점(2.95,-20), 미관측(-1) 없음.
- 첫/마지막 도로값0은 426/424셀, 전방17.6375~25.45m 및 횡방향4.0625~14.375m의 작은 영역에만 존재.
- 추가 12초 161프레임에서 도로 bbox는 끝 열만70~71로 변동. 모든 프레임에서 그리드 처음9m 구간의 도로셀0개.
- 송신 노드의 조회 가능한 파라미터는 이전16:51 조회와 동일(analysis.json 참조).
- 상위 플래너 노드는 이번 관측에 없었고 계획 이벤트/플래너 로그도 수신되지 않음. 현재 QP 성공 여부는 판정 불가.
- 메시지 수신은 확인됐지만 근거리부터 이어지는 도로는 확인되지 않음. 40m 도로 인식 해결로 판정할 수 없음.
- 앞서 사용하던 origin override=0을 적용하면 위 도로 x범위는14.6875~22.5m. 현재 상위 실행 파라미터로 확인된 것은 아님.
- 원인이 영상/투영/TF/도로 분류 중 무엇인지는 이 점검만으로 확정하지 않음.
- 제어 송신, 주행, ARM, 운영 소스/설정 변경 없음.
'''
(p/'RESULT.txt').write_text(report)
