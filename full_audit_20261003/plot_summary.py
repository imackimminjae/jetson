from pathlib import Path
import json,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
plt.rcParams.update({'font.family':FontProperties(fname='/usr/share/fonts/truetype/nanum/NanumGothic.ttf').get_name(),'axes.unicode_minus':False})
p=Path(__file__).resolve().parent;rows=[json.loads(s) for s in (p/'results.jsonl').read_text().splitlines()];fig,axs=plt.subplots(3,1,figsize=(12,11))
for name,label,c in [('current','차량 0.74초 / 제어모델 0.05초','#c62828'),('matched_model','차량 0.74초 / 제어모델 0.74초','#087e8b'),('extra_lpf','불일치 유지 + 출력 저역통과필터','#9c6a20')]:
 r=next(x for x in rows if x['scene']=='fixed_curve' and x['plant']['name']=='measured_slow' and x['config']['name']==name);s=r['states'];axs[0].plot([x['t'] for x in s],[x['ey'] for x in s],label=label,c=c)
axs[0].set(title='고정된 곡선 경로에서도 응답 불일치만으로 진동 발생 — 수치 실험',xlabel='시간 (초)',ylabel='횡오차 (m)');axs[0].legend(fontsize=9);axs[0].grid(alpha=.2)
metrics=json.load(open(p/'common_progress_metrics.json'));xs=np.arange(2);barw=.32
for off,name,label,col in [(-barw/2,'current','현재 모델','#c62828'),(barw/2,'matched_model','응답 일치','#087e8b')]:
 vals=[next(x['lateral_p95_m'] for x in metrics if x['scene']==scene and x['config']==name) for scene in ['live_curve','branch_width']];axs[1].bar(xs+off,vals,width=barw,label=label,color=col)
 for x,v in zip(xs+off,vals):axs[1].text(x,v+.025,f'{v:.2f}m',ha='center')
axs[1].set(xticks=xs,xticklabels=['곡선','분기·폭 변화'],ylabel='횡오차 95백분위 (m)',title='지도→계획→제어→차량 연결 실험 / 같은 진행 거리까지만 비교');axs[1].legend(fontsize=9);axs[1].set_ylim(0,1.9)
raw=17.465557260937192
for y,B,label in [(1,11.5,'직전 B를 유지한 가상조건'),(0,14.8,'실제 갱신된 B')]:
 inset=.3*raw if raw<=1.5*B else 2.;width=raw-2*inset
 axs[2].plot([-raw/2,raw/2],[y,y],c='#d8d8d8',lw=15);axs[2].plot([-width/2,width/2],[y,y],c='#087e8b' if y else '#c62828',lw=15);axs[2].text(0,y+.18,f'B={B:.1f}m / 기준 1.5B={1.5*B:.2f}m / 허용폭 {width:.2f}m',ha='center')
axs[2].set(yticks=[0,1],yticklabels=['갱신 후','직전 B 유지'],ylim=(-.5,1.6),xlabel='같은 단면의 중심에서 거리 (m)',title='05:59:42 첫 실패의 제약 재현: 같은 관측폭인데 B 갱신으로 허용폭 급감')
fig.suptitle('실제 로그 제약 재현 + 독립 수치 실험\n실차/Isaac 전체 재현이 아니며, 운영 설정은 변경하지 않음',fontsize=14);fig.tight_layout();fig.savefig(p/'audit_summary.png',dpi=155)
