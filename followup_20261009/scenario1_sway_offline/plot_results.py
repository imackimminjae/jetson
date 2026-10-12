from pathlib import Path
import json,gzip,os
os.environ['MPLCONFIGDIR']='/tmp/scenario1_sway_offline_plot'
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
font_manager.fontManager.addfont('/usr/share/fonts/truetype/nanum/NanumGothic.ttf')
plt.rcParams.update({'font.family':'NanumGothic','axes.unicode_minus':False,'font.size':11})
H=Path(__file__).resolve().parent;s=json.loads((H/'summary.json').read_text());cc=s['critical']
labels={'fixed':'기록 재현','B7_4':'B만 7.4m','soft_k1':'k1만 soft 해제','relax_only':'넓은 구간만 완화'}
colors={'fixed':'#bc4b42','B7_4':'#21868f','soft_k1':'#a28ab0','relax_only':'#539447'}
chosen=[c for c in cc if c['attempt'] in [1,3,8]]
fig,axs=plt.subplots(1,2,figsize=(14,5),layout='constrained');x=np.arange(3)
for j,mode in enumerate(labels):
    for ax,key in zip(axs,['k1_shift','output_deg']):
        ax.bar(x+(j-1.5)*.19,[c['modes'][mode][key] for c in chosen],width=.18,color=colors[mode],label=labels[mode])
for ax in axs:ax.set_xticks(x,[f'{n}회' for n in [1,3,8]]);ax.grid(axis='y',alpha=.2)
axs[0].axhline(0,c='#888',lw=.8);axs[0].set(title='같은 참조에서 새 k1의 횡방향 이동',ylabel='직전 발행 계획 기준 (m), 왼쪽 +')
axs[1].set(title='같은 하위 상태에서 출력 조향 명령',ylabel='명령 환산값 (도, 실제 바퀴각 아님)')
axs[0].legend(fontsize=9);fig.suptitle('분기 직전 기록 상태 고정 비교: B/soft 처리가 계획·조향에 미치는 영향',fontsize=16)
fig.supxlabel('상위: 각 회차 Y≈-22.5m 계산 / 하위: 그 계획 사용 구간의 기준 명령 최대 표본\n각 대조는 기록 위치·방향·속도·직전 명령을 고정한 별도 계산이며, 변경 후 실제 주행 궤적이 아님',fontsize=10)
fig.savefig(H/'causal_comparison.png',dpi=155);plt.close(fig)

fig,axs=plt.subplots(2,5,figsize=(19,9),layout='constrained')
for c,ax in zip(cc,axs.flat):
    n=c['attempt'];j=c['cycle'];ip=json.loads((H/f'input_{n}.json').read_text())['cycles'][j];r=json.load(gzip.open(H/f'upper_{n}.json.gz','rt'));ref=np.array(ip['rec_preview'])
    ax.plot(*ref.T,'--',c='#777',label='고정한 참조')
    for mode in ['fixed','B7_4']:
        z=np.array(r[mode][j]['published']);ax.plot(*z.T,'o-',ms=3,color=colors[mode],label=labels[mode])
    ax.scatter(ip['x'],ip['y'],c='black',marker='^',s=35)
    ax.set(xlim=(110,138),ylim=(-25,1),title=f'{n}회'+(' · 기록상 오분기' if n==9 else ''),xlabel='X (m)',ylabel='Y (m)');ax.set_aspect('equal');ax.grid(alpha=.2)
axs.flat[0].legend(fontsize=8);fig.suptitle('시나리오1 전 10회: 같은 상태·같은 참조에서 B만 바꾼 계획 비교\n이후 실제 주행 결과를 예측한 그림이 아님',fontsize=15)
fig.savefig(H/'all10_fixed_state_plans.png',dpi=145);plt.close(fig)

tags=['latest_sc1_8','reg_sc3_batch04','reg_sc3_batch05','reg_sc3_batch06','reg_sc3_batch07','reg_20261006_050536']
names=['최신 시나리오1\n8회','06:26 시나리오3\n4회','06:26 시나리오3\n5회','06:26 시나리오3\n6회','06:26 시나리오3\n7회','10/06 시나리오1\n05:05 오분기 기록']
fig,ax=plt.subplots(figsize=(13,5),layout='constrained');xx=np.arange(len(tags))
for j,mode in enumerate(['fixed','B7_4','relax_only']):
    vals=[next(v for v in s['comparisons'] if v['tag']==tag and v['mode']==mode)['variant_fail'] for tag in tags]
    bars=ax.bar(xx+(j-1)*.24,vals,width=.23,color=colors[mode],label=labels[mode]);ax.bar_label(bars,fontsize=9)
ax.set_xticks(xx,names);ax.set_ylabel('기록 상태별 상위 계획 실패 횟수');ax.grid(axis='y',alpha=.2);ax.legend()
ax.set_title('회귀 대조: 문제 지점의 조향 감소가 전체 구간의 개선을 뜻하지 않음',fontsize=15)
fig.supxlabel('기록 상태·참조 고정 / 4·6회 기존 후보 선택 유지 / 5회 오분기 방향은 해소되지 않음\n넓은 구간만 완화한 후보도 과거 시나리오1에서 새 실패 1회 발생 → 운영 적용 보류',fontsize=10)
fig.savefig(H/'regression_comparison.png',dpi=155);plt.close(fig)
print('plots saved')
