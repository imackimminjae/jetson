from pathlib import Path
import os,json,gzip
os.environ['MPLCONFIGDIR']='/tmp/B_lineage_plot'
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from matplotlib import font_manager
font_manager.fontManager.addfont('/usr/share/fonts/truetype/nanum/NanumGothic.ttf');plt.rcParams.update({'font.family':'NanumGothic','axes.unicode_minus':False,'font.size':10})
H=Path(__file__).resolve().parent;P=H.parent/'scenario1_sway_offline';audit=json.loads((H/'audit.json').read_text());rows=audit['cycles_detail']
gridbytes=gzip.decompress((P/'grids.bin.gz').read_bytes())
colors={1:'#bc4b42',3:'#d38c35',8:'#b55892',5:'#208546',6:'#278fa6',10:'#6064b0'}
fig,axs=plt.subplots(1,3,figsize=(18,6),layout='constrained')
for n in [1,3,8,5,6,10]:
    q=[z for z in rows if z['attempt']==n and -40<z['y']<-14];axs[0].step([z['y'] for z in q],[z['B'] for z in q],where='post',color=colors[n],label=f'{n}회')
axs[0].set(xlabel='차량 Y (m)',ylabel='B (m)',title='두 분기 사이 B의 선택 이력');axs[0].grid(alpha=.2);axs[0].legend(ncol=3)
for n,ax in zip([1,5],axs[1:]):
    k=next(q for q in audit['key'] if q['attempt']==n)['critical'];j=k['cycle'];c=json.loads((P/f'input_{n}.json').read_text())['cycles'][j];z=json.load(gzip.open(P/f'upper_{n}.json.gz','rt'))['fixed'][j];yaw=c['yaw'];rot=np.array([[np.cos(yaw),-np.sin(yaw)],[np.sin(yaw),np.cos(yaw)]]);pos=np.array([c['x'],c['y']]);gi=c['grid_index'];g=np.frombuffer(gridbytes[gi*16384:(gi+1)*16384],dtype=np.int8).reshape(128,128)
    x,y=np.meshgrid(np.arange(129)*.3125,np.arange(129)*.3125-20);xy=np.stack([x,y],axis=-1)@rot.T+pos
    status=np.where(g<0,2,np.where(g<=1,0,1));ax.pcolormesh(xy[:,:,0],xy[:,:,1],status,cmap=ListedColormap(['#f4f5ed','#a9afb1','#e5dcbd']),vmin=0,vmax=2,shading='flat',rasterized=True)
    ref=np.array(c['rec_preview']);ax.plot(*ref.T,'--o',color='#69615f',ms=3,label='이어받은 참조')
    for q in z['raw_detail']:
        if q['k']!=5:continue
        raw=np.array(q['raw_body']).reshape(2,2)@rot.T+pos;length=np.linalg.norm(raw[1]-raw[0]);col='#217b55' if q['inside'] else '#d08c2e'
        ax.plot(*raw.T,color=col,lw=3,label='참조 포함 k5 구간' if q['inside'] else '참조 미포함 k5 구간');mid=raw.mean(axis=0);ax.text(mid[0]+.2,mid[1]+.4,f'{length:.1f}m',fontsize=10,color=col,bbox=dict(facecolor='white',alpha=.8,edgecolor='none',pad=1))
    ax.scatter(*ref[5],s=100,c='#b32332',marker='*',zorder=8,label='k5 참조점');ax.scatter(125.7,-4.6,s=230,facecolors='none',edgecolors='#c51f30',lw=1.5,label='단면 분리 차이 부근')
    ax.set(xlim=(114,138),ylim=(-14,9),xlabel='X (m)',ylabel='Y (m)',title=f"{n}회 · B={c['rec_B']:.1f}m (k{z['B_candidate_step']} 선택)\n기록 BEV: 밝은색 통행 가능 / 회색 막힘")
    ax.set_aspect('equal');ax.grid(alpha=.15);ax.legend(fontsize=8,loc='upper right')
fig.suptitle('B가 늦게 낮아진 이유: 참조 단면이 분기 사이 막힌 셀을 통과하는 시점 차이',fontsize=16)
fig.supxlabel('1회 k5는 18.5m 연속 구간이라 더 짧은 k2=14.0m 선택 / 5회 k5는 분리된7.4m 구간이 참조를 포함해 B로 선택\n회차 간 지도·상태가 다른 관측 비교이며, 동일 지도에서 단면만 바꾼 대조는 RESULT.txt에 별도 기록',fontsize=10)
fig.savefig(H/'B_lineage.png',dpi=150);plt.close(fig)
fig,axs=plt.subplots(2,5,figsize=(19,9),layout='constrained')
for n,ax in enumerate(axs.flat,1):
    q=[z for z in rows if z['attempt']==n and -60<z['y']<-12];key=next(z for z in audit['key'] if z['attempt']==n)
    ax.step([z['y'] for z in q],[z['B'] for z in q],where='post',c=colors.get(n,'#777'))
    for z in q:
        if -30<z['y']<-14:ax.annotate(f"k{z['candidate_k']}",(z['y'],z['B']),xytext=(0,5),textcoords='offset points',ha='center',fontsize=7)
    ax.axvline(key['high']['y'],color='#ba5555',ls=':',lw=1);ax.axvline(key['drop']['y'],color='#238546',ls=':',lw=1)
    ax.set(xlim=(-60,-12),ylim=(0,17),xlabel='차량 Y (m)',ylabel='B (m)',title=f"{n}회 · B≥13m 약{key['high_duration_sec']:.1f}초\n감소: k{key['drop']['candidate_k']}={key['drop']['B']:.1f}m")
    ax.grid(alpha=.2)
fig.suptitle('시나리오1 전10회: 첫 분기 이전부터 B와 선택 단계 추적\n빨간 점선: 이 구간에서13m 이상 최초 선택 / 초록 점선: 이후10m 미만 선택',fontsize=15)
fig.savefig(H/'all10_B_history.png',dpi=145);plt.close(fig)
print('figures saved')
