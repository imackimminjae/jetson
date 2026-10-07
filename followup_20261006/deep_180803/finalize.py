from pathlib import Path
import json,pickle,hashlib,os,numpy as np
os.environ['MPLCONFIGDIR']='/tmp/codex_mpl_grid'
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from matplotlib import font_manager
font_manager.fontManager.addfont('/usr/share/fonts/truetype/nanum/NanumGothic.ttf');plt.rcParams.update({'font.family':'NanumGothic','axes.unicode_minus':False})
H=Path(__file__).resolve().parent;R=H.parents[1];C=json.loads((H/'input.json').read_text())['cycles'];D=json.loads((H/'result.json').read_text());X={**json.loads((H/'refB_result.json').read_text()),**json.loads((H/'freshB_result.json').read_text())};d=pickle.load((R/'drive_debug_20261006_180803_light/analysis/decoded.pkl').open('rb'))
validmatch=sum(c['rec_valid']==r['valid'] for c,r in zip(C,D['baseline']));errors=[{'i':i,'max_m':float(np.linalg.norm(np.array(c['rec_planned'])-np.array(r['planned']),axis=1).max())} for i,(c,r) in enumerate(zip(C,D['baseline'])) if c['rec_valid'] and r['valid'] and len(c['rec_planned'])==len(r['planned'])]
summary={'bag':'180803','validity_matches':validmatch,'cycles':len(C),'valid_plans_under_1mm':sum(e['max_m']<.001 for e in errors),'valid_plans':len(errors),'residuals_over_1mm':[e for e in errors if e['max_m']>=.001],'valid_by_variant':{k:sum(v['valid'] for v in z) for k,z in D.items()},'reference_B_cross_at_23_726':{k:{'B':v[34]['B'],'end':v[34]['planned'][-1]} for k,v in X.items()},'source_hashes_unchanged':all(hashlib.sha256(Path(k).read_bytes()).hexdigest()==v for k,v in json.loads((H/'sources_sha256.json').read_text()).items())};(H/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2))
i=34;c=C[i];g=d['/bev/occupancy_grid'][c['grid_record_index']];r=g['res'];x,y=np.meshgrid(np.arange(129)*r,np.arange(129)*r-20);a=c['yaw'];gx=c['x']+np.cos(a)*x-np.sin(a)*y;gy=c['y']+np.sin(a)*x+np.cos(a)*y
fig,axs=plt.subplots(1,2,figsize=(13,6),layout='constrained')
for ax,k,title in zip(axs,['recorded_ref_recorded_B','fresh_ref_recorded_B'],['기존 참조 + B 6.1 m','새 참조 + 동일 B 6.1 m']):
 z=X[k][i];ax.pcolormesh(gx,gy,g['grid'],cmap=ListedColormap(['#e1f0e4','#f0d8d8']),vmin=0,vmax=100,rasterized=True);gp=d['/navigation/global_path'][0]['xy'];ax.plot(*gp.T,'k--o',ms=3,label='지정 경로');ax.plot(*np.array(z['reference']).T,':',c='#bc7f21',lw=2,label='단면 추출용 참조');ax.plot(*np.array(z['planned']).T,'o-',c='#1963bc',ms=4,lw=2,label='계산된 계획')
 for k2 in range(1,len(z['cands'])):
  for iv in z['cands'][k2]:ax.plot([iv[0],iv[2]],[iv[1],iv[3]],c='#666666',lw=.8,alpha=.7)
 ax.plot(c['x'],c['y'],'ko',ms=7,label='동일한 차량 위치');ax.set(xlim=(241,285),ylim=(120,176),xlabel='지도 X (m)',ylabel='지도 Y (m)',title=title);ax.set_aspect('equal');ax.legend(fontsize=8,loc='upper left')
fig.suptitle('18:08:23.726 · 참조만 바꾸면 다른 갈래의 계획이 가능\n같은 수신 지도·차량 상태 / 회색: 허용 단면 / 오프라인 상위 계산, 실제 주행 검증 아님');fig.savefig(H/'reference_B_comparison.png',dpi=150);plt.close(fig)
text='''18:08:03 기록 상세 원인 분석 (후속 18:18/18:19 기록과 구분)

핵심
이전 계획을 참조로 이어받아 단면을 추출하는 구조에서, 남쪽으로 이어진 참조가 정분기 방향의 가능한 계획을 제한한다. 분기 후보가 모두 하나이면 큰 도로 회전 요구가 있어도 회전 비용이 생성되지 않는다. 마지막 위치 비용의 목표도 기존 참조 끝에서 한 걸음 이동한 국소 목표다. 따라서 높은 terminal 가중치만으로 전역 경로를 강제하는 구조가 아니다.

근거와 시계열
- 18:08:20.226: wp5→6, 필요한 도로 회전 76.18도, 모든 미래 단면 후보 하나, 회전 비용 비활성. 계획 끝 (256.8,136.3), 마지막 위치 목표 (256.1,139.4).
- 18:08:21.726: 기존 단면/입력 제약 하에서 x=255~280 내 도달 가능한 끝점 Y의 최대 133.12. 축소/경계 여유를 모두 제거해도 134.36. 단순 비용 조정으로 북동쪽 끝점 Y>=145를 만들 수 없음.
- 18:08:23.726: 차량 (247.88,138.20), wp5=(255,135)에 도착하기 전에 wp pair 5/6→6/7. 도로 회전량 76.18→3.37도. 기존 참조는 유지. 후보 모두 하나, 계획 끝 (277.02,133.13).
- 18:08:29.726에 N6, 이후 N7 복귀 시 참조 차원 불일치/초기화 후 연속 실패. 이 현상은 이미 오분기한 뒤 발생한 후속 문제.

참조/B 분리 (같은 18:08:23.726 입력)
- 기존 참조 + 기록 B=6.1: 끝 (277.02,133.13).
- 기존 참조 + 새 참조에서 얻은 B=8.7: 끝 동일.
- 새 참조 + 기록 B=6.1: 끝 (269.79,162.12).
- 새 참조 + B=8.7: 끝 동일.
B 변화 없이 참조를 바꿔 북동쪽 계획을 생성할 수 있다. 이 비교에서 B만 변경한 효과는 없다. 모든 시점/다른 기록에서 B가 무관하다는 뜻은 아니다.

진단적 최소 변경 대조
r_dpsi 3→2, B=5 고정, 회전 가중치 10배, 회전 창 k4 강제, wp5 유지, wp5 유지+회전 창 강제, 단면 축소/경계 여유 제거: 위 핵심 시점에서 북동쪽 계획을 복구하지 못함.
참조 매회 초기화는 핵심 두 시점에서 북동쪽 계획을 만들지만 전체 51개 독립 입력 중 유효 계산 47→36으로 감소한다. 운영 적용안이 아니며 채택 보류. 다른 시점에는 남쪽 계획도 지속한다.

재현성/한계
생산 소스 복사본 빌드. 도메인177/localhost, timer 취소, spin 없음. 51×9 + 51×4=663회 독립 상위 계산. 기록된 차량 위치/지도 고정. 이전 변경 계획이 다음 차량 위치를 바꾸는 폐루프가 아니다. 새 참조는 기존 코드의 fresh straight seed 경로이며, 참조의 위치·방향과 그로부터 나온 단면/비용 목표 전체가 함께 달라진다. 참조 내부 각 요소의 효과까지 분리한 것은 아니다.
기준 재현: 유효/실패 51/51 일치, 유효 계획 47개 중46개 1mm 이내. N6으로 줄어든 마지막 유효 계획 한 개는 최대0.171m 차이. 원인 분석 핵심 N7 구간은 재현됨.
가장 가까운 이전 bag 수신 지도 사용. 위 핵심 시점 지도 수신차 17.44ms. 그림은 운영의 effective origin x=0을 사용, 수신 origin 2.95의 물리적 정합을 입증하지 않음.
LP는 생산 선형화 위치/입력 제약에서 모든 후보 조합의 실행 가능성을 검사한다. NE box x255~280,y145~170는 진단 영역이며 도로 전체/차량 동역학 통과의 증명이 아니다.
원본 bag, 운영 소스, 설정 변경 없음. 주행/ARM/하드웨어 변경 없음.

파일
result.json: 9종 대조. refB_result.json, freshB_result.json: 2×2 참조/B 대조.
feasibility_check.py / feasibility.json: LP 제약 검증.
recorded_timeline.json, summary.json, reference_B_comparison.png.
prepare.py/build.py/replay.cpp/diagnostic.patch: 입력 추출과 격리 빌드 근거.
재실행: install/setup.bash 로드 후 ROS_DOMAIN_ID=177 ROS_LOCALHOST_ONLY=1 ROS_LOG_DIR을 이 폴더/ros_logs로 지정, replay input.json grids.bin config.yaml result.json specs.json.
'''
(H/'RESULT.txt').write_text(text)
print(json.dumps(summary,ensure_ascii=False,indent=2))
