from pathlib import Path
import json,csv,hashlib,math
import numpy as np
import matplotlib
matplotlib.use('Agg')
from matplotlib import pyplot as plt,font_manager
from matplotlib.colors import ListedColormap,BoundaryNorm
O=Path(__file__).resolve().parent;P=O.parent/'scenario2_short_041941_review'
records=json.load(open(P/'selected_branch_cycles.json'));results=json.load(open(O/'results.json'));case=records[1];e=case['event'];t=case['upper_trace']
def get(lam,force):return next(r for r in results if r['attempt']==2 and r['lambda']==lam and r['forced_last']==force)
font_manager.fontManager.addfont('/usr/share/fonts/truetype/nanum/NanumGothic.ttf')
plt.rcParams.update({'font.family':'NanumGothic','axes.unicode_minus':False})
fig,axes=plt.subplots(1,2,figsize=(13,6.5));ax=axes[0]
grid=np.load(P/f'scenario2/case_a2_seq{e["plan_seq"]}_grid.npz')['data']
x,y=np.meshgrid(np.arange(129)*.3125,np.arange(129)*.3125-20)
c,s=np.cos(e['yaw_rad']),np.sin(e['yaw_rad']);wx=e['x']+c*x-s*y;wy=e['y']+s*x+c*y
cats=np.where(grid<0,2,np.where(grid<=1,0,1));ax.pcolormesh(wx,wy,cats,cmap=ListedColormap(['white','#575757','#e8cf8b']),norm=BoundaryNorm([-.5,.5,1.5,2.5],3),shading='flat',rasterized=True)
for w in e['intervals_world']:ax.plot([w[2],w[4]],[w[3],w[5]],color='#23995e',lw=1,alpha=.7)
pr=np.array(e['preview_world']);ax.plot(pr[:,0],pr[:,1],':',color='#cb870c',label='고정한 이전 참조',lw=2)
for lam,col in [(.6,'#d13a45'),(1.2,'#eb8e3a'),(2.0,'#177ac2')]:
 p=np.array(get(lam,-1)['predicted_world']);ax.plot(p[:,0],p[:,1],'-o',ms=3,color=col,lw=2,label=f'헤딩 가중치 {lam:g}')
ax.plot(e['x'],e['y'],'k^',ms=9)
a=t[12]+t[3];ax.annotate('',xy=(e['x']+10*math.cos(a),e['y']+10*math.sin(a)),xytext=(e['x'],e['y']),arrowprops=dict(arrowstyle='->',color='#723da4',lw=2))
ax.text(-143,61,'요구 출구 방향 +10.9°',fontsize=8,color='#723da4')
ax.set_xlim(-150,-110);ax.set_ylim(42,76);ax.set_aspect('equal');ax.set_xlabel('map X [m]');ax.set_ylabel('map Y [m]');ax.grid(alpha=.15);ax.legend(fontsize=9,loc='upper left');ax.set_title('실패 2회차 · 출발 후 24.83초\n같은 BEV·인터벌·참조에서 가중치만 변경')
ax=axes[1];selected=[get(l,f) for l in [.6,2.] for f in [0,1]];pos=np.arange(4);bottom=np.zeros(4)
for key,label,col in [('position_cost','위치 비용','#7b9bc5'),('input_cost','입력 보정 비용','#dbb756'),('turn_cost','헤딩 비용','#be7393')]:
 vals=np.array([r[key] for r in selected]);ax.bar(pos,vals,bottom=bottom,color=col,label=label)
 for i,v in enumerate(vals):ax.text(i,bottom[i]+v/2,f'{v:.3f}',ha='center',va='center',fontsize=9)
 bottom+=vals
for i,total in enumerate(bottom):ax.text(i,total+.06,f'합계 {total:.3f}',ha='center',fontsize=9)
ax.set_xticks(pos,['0.6 / 아래','0.6 / 위','2.0 / 아래','2.0 / 위']);ax.set_ylabel('정규화된 비용');ax.set_ylim(0,4.2);ax.legend(fontsize=9);ax.grid(axis='y',alpha=.15);ax.set_title('갈래별 최소 비용 비교\n0.6은 아래 선택 → 2.0은 위 선택')
fig.suptitle('시나리오2 · 헤딩 비용을 높이면 위쪽으로 바뀌는가?',fontsize=15)
fig.text(.055,.018,'저장 로그의 한 계산 주기를 재계산한 결과. 파란 선은 실제 주행이 아닌 반사실 계획. 운영 설정 변경 없음.\nBEV는 기존 분석의 선행 수신 단일 프레임이며, 사용 grid ID가 기록되지 않아 완전한 동시각 대응은 보장되지 않음.',fontsize=8)
fig.tight_layout(rect=[0,.075,1,.93]);fig.savefig(O/'heading_cost_comparison.png',dpi=170);fig.savefig(O/'heading_cost_comparison.pdf');plt.close(fig)
fields=['attempt','epoch','lambda','forced_last','valid','position_cost','input_cost','turn_cost','terminal_error','branch_step','status']
with open(O/'costs.csv','w',newline='') as f:
 w=csv.DictWriter(f,fieldnames=fields);w.writeheader();w.writerows({k:r[k] for k in fields} for r in results)
# Every protected input must remain unchanged.
hashes=json.load(open(O/'input_hashes.json'));assert all(hashlib.sha256(Path(p).read_bytes()).hexdigest()==h for p,h in hashes.items())
v=json.load(open(O/'validation.json'));v['protected_inputs_unchanged']=True;v['ros_nodes_started']=False;v['additional_grid_frames_decoded']=0
json.dump(v,open(O/'validation.json','w'),indent=2)
text='''시나리오2 · 실패 2회차의 헤딩 비용 분석
배치: 20261013_041941_scenario2_173ca0 (단축한 5점 경로)
대표 시점: 출발 후 24.83초, map(-144.3293,58.3354), 차량 yaw -11.94도.

결론: 대표 시점의 같은 입력에서는 lambda_relative_turn 1.5까지 아래쪽,
1.6부터 위쪽 인터벌을 선택한다. 0.9/1.2로는 선택이 바뀌지 않는다.
이는 저장한 한 계산 주기의 반사실 결과이며, 실제 반복 주행의 성공 보장이 아니다.

현재 0.6에서 아래쪽: 위치0.100924 + 입력0.277069 + 헤딩0.944307 = 1.322300.
같은 0.6에서 위쪽을 제약으로 고정한 최소해:
위치1.1269 + 입력0.6238 + 헤딩0.4754 = 약2.2261.
위쪽은 헤딩 비용이 낮지만, 아래로 휘어진 이전 참조에 대한 위치·입력 보정 비용이 크다.
따라서 현재 비용 균형에서는 아래쪽 최소해를 선택한다. 위쪽 경로 자체는 이 시점에 feasible.

가중치2.0: 아래쪽 합계 약3.4929, 위쪽 합계 약3.0150 → 위쪽 선택.
성공 1회차의 같은 위치 인근에서는 0.6→2.0에서 갈래 선택이 유지된다.

헤딩 항은 하위 제어기의 추종 오차 가중치가 아니라 상위 MIQP의 상대회전 비용이다.
현재 분기 검출 결과는 k5이며 헤딩 비용은 k3,k4,k5에 적용되고 단계 비중은1/7,2/7,4/7.
요구 map 방향은 각각 -6.24,+2.34,+10.92도.
실패 계획의 모델 map 헤딩은 -18.49,-28.86,-30.06도, 마지막 창의 오차는 -40.98도.
그 뒤 k6,k7의 헤딩에는 이 항이 직접 적용되지 않는다.
비용의 입력 항은 참조에 대한 dv,dpsi 보정량이며 실제 바퀴각 측정 비용이 아니다.
trace의 objective는 상수항을 생략하므로 위 세 보고 비용의 합과 직접 같지 않다.

인접 13개 계산 주기(성공1회 7개, 실패2회 6개)도 재계산했다.
기존0.6 해의 모든 좌표와 세 비용 항이 기록에 일치:
최대 좌표오차 2.85e-14m 이하, 최대 비용오차 2.56e-15 이하.
실패2회 25.33초에는 마지막 단계 후보가1개로, 재계산해도 위쪽 갈래로 바뀌지 않는다.
이때는 이전 참조/그 참조 위에서 추출한 후보가 이미 아래쪽으로 이어지므로
가중치 증가만으로 모든 순간에 위쪽을 선택한다고 말할 수 없다.

방법: 실제 C++ solveUpperMiqp/applyTerminalPositionTargets를 오프라인 전용 프로그램으로
가져와 같은 MIQPLib/DAQP를 사용. ROS 노드/실주행 없음.
pose,속도6m/s,참조,후보,제약,목표를 고정하고 헤딩 가중치만 변경.
갈래별 비교에는 마지막 후보만 추가로 고정, 나머지 단계 조합은 모두 탐색.
대표 시점의 마지막 후보0=아래,1=위. 후보 번호는 다른 시점과 동일한 의미라고 가정하지 않음.
운영 설정·소스·경로 변경 없음. 기존 단일 BEV를 재사용하고 추가 디코딩 없음.

권고: 2.0은 비교 시험 후보로 근거가 있다. 적용 전 다른 분기에서도 같은 비용 비교가 필요하며,
전체 반복 주행 결과로 효과 확인 필요. 이번 요청에서는 운영 가중치를 적용하지 않았다.
'''
(O/'RESULT_KO.txt').write_text(text)
print('그림·비용표·분석 저장, 입력 보존 확인 완료')
