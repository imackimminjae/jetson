from pathlib import Path
import json,numpy as np
from scipy.signal import butter,sosfiltfilt
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
p=Path(__file__).resolve().parent
runs=[json.loads(x) for f in ['results.jsonl','injected_results.jsonl','refine_results.jsonl'] for x in (p/f).read_text().splitlines()]
lookup={(r['scene'],r['plant']['name'],r['config']['name']):r for r in runs}
metrics=json.loads((p/'metrics.json').read_text())['metrics']
def mean(scene,name,k):return np.mean([r[k] for r in metrics if r['scene']==scene and r['config']==name])
font_manager.fontManager.addfont('/usr/share/fonts/truetype/nanum/NanumGothic.ttf')
plt.rcParams.update({'font.family':'NanumGothic','axes.unicode_minus':False,'font.size':10})
fig,axs=plt.subplots(2,2,figsize=(14,10),layout='constrained')
for ax,scene,candidate,title,limits in [
    (axs[0,0],'recorded_front','rd240','실제 앞쪽 계획 입력 → 가상 차량 조향',(4,16.4)),
    (axs[0,1],'injected_sway','rd240','알려진 좌우 경로 흔들림을 입력한 시험',(8,18)),
    (axs[1,0],'live_curve','dpsi2_rd240','상·하위가 함께 갱신되는 가상 주행',(20,34))]:
    for name,color,label in [('baseline','#444','현재 설정'),(candidate,'#087fb8','하위 변화 비용 240' if candidate=='rd240' else '상위 방향 비용 2 + 하위 변화 비용 240')]:
        r=lookup[(scene,'perturbed',name)];s=r['states'];t=np.array([z['t'] for z in s]);cmd=np.degrees([z['command'] for z in s])
        ax.plot(t,cmd,c=color,label=label,lw=1.6)
    ax.set_xlim(*limits);ax.set_title(title);ax.set_xlabel('가상 시간 (s)');ax.set_ylabel('조향 명령 (deg)');ax.grid(alpha=.2);ax.legend(fontsize=9)
ax=axs[1,1];cases=[('recorded_front','rd240','기록 경로 입력\n하위 240'),('injected_sway','rd240','좌우 흔들림 주입\n하위 240'),('live_curve','dpsi2_rd240','상·하위 폐루프\n상위 2 + 하위 240')]
values=[100*(mean(scene,name,'hp03_rms_deg')/mean(scene,'baseline','hp03_rms_deg')-1) for scene,name,label in cases]
bars=ax.bar([x[2] for x in cases],values,color=['#339ac5','#339ac5','#12698d']);ax.bar_label(bars,fmt='%.1f%%',padding=4);ax.set_ylim(-75,0);ax.set_ylabel('0.3 Hz 이상 조향 RMS 변화 (%)');ax.set_title('두 차량 응답 조건 평균 / 각각의 기준 설정과 비교');ax.grid(axis='y',alpha=.2)
fig.suptitle('186회 조향 떨림 중심 시험: 출력 필터보다 MPC 내부 변화 비용이 일관된 후보',fontsize=15)
fig.supxlabel('실제 계산 코드와 가상 자전거 모델. 0.3 Hz 이상 성분은 반복 흔들림의 근사 지표이며 실제 차량 개선율이 아님.',fontsize=10)
fig.savefig(p/'steering_focus_summary.png',dpi=150)
print('reductions',values)
