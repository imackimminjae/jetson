import json,numpy as np,matplotlib;matplotlib.use('Agg');import matplotlib.pyplot as plt
from matplotlib import font_manager
font_manager.fontManager.addfont('/usr/share/fonts/truetype/nanum/NanumGothic.ttf');plt.rcParams.update({'font.family':'NanumGothic','axes.unicode_minus':False})
sm=json.load(open('static_map.json'));M=np.fromfile('static_map.bin',dtype=np.int8).reshape(sm['height'],sm['width']);ext=(sm['origin'][0],sm['origin'][0]+sm['width']*sm['res'],sm['origin'][1],sm['origin'][1]+sm['height']*sm['res'])
R=[json.loads(l) for l in open('cl_results_v3.jsonl')]
show=[('base','현재 (base)'),('gated-split','제한형 split (gated-split)')]
fig,ax=plt.subplots(3,2,figsize=(16,13),gridspec_kw=dict(height_ratios=[1.1,1,1]))
for j,(v,title) in enumerate(show):
    rs=[r for r in R if r['config']['name']==v and r['plant']['name']=='fit_tau0.74_d0.1']
    ax[0,j].imshow(M,origin='lower',cmap='gray_r',extent=ext,alpha=.5)
    nosc=0
    for r in rs:
        S=r['states'];x=np.array([s['x'] for s in S]);y=np.array([s['y'] for s in S]);c=np.degrees([s['cmd'] for s in S]);e=(x>=120)&(y>-20)
        osc=e.any() and (np.mean(abs(c[e])>=17)*100>5 or c[e].std()>9);nosc+=osc
        col='crimson' if r['outcome']=='north_branch' else ('tab:red' if osc else 'tab:blue');lw=1.4 if osc or r['outcome']!='east_done' else .7
        ax[0,j].plot(x,y,color=col,lw=lw,alpha=.8);ax[1,j].plot(x,c,color=col,lw=lw,alpha=.8)
        P=r['plans'];ax[2,j].plot([p['x'] for p in P],[p['B'] for p in P],color=col,lw=lw,alpha=.7,drawstyle='steps-post')
    wrong=sum(r['outcome']=='north_branch' for r in rs)
    ax[0,j].set_title(f"{title} — 25회 중 진동 {nosc}회, 북쪽 갈래 {wrong}회 (빨강=진동/오분기)")
    ax[0,j].set_xlim(105,170);ax[0,j].set_ylim(-35,15);ax[0,j].set_aspect('equal');ax[0,j].set_ylabel('Y (m)')
    ax[1,j].set_xlim(105,170);ax[1,j].set_ylim(-19,19);ax[1,j].set_ylabel('출력 조향 명령 (도)');ax[1,j].axhline(17.1,ls=':',c='k');ax[1,j].axhline(-17.1,ls=':',c='k')
    ax[2,j].set_xlim(105,170);ax[2,j].set_ylabel('B (m)');ax[2,j].set_xlabel('X (m)')
    for a in ax[:,j]: a.grid(alpha=.25)
fig.suptitle('시나리오1 폐루프 시뮬 (기록 BEV 재투영, 플랜트 τ=0.74s·지연0.1s, 시작점5×위상5)',fontsize=13)
fig.savefig('comparison_s1.png',dpi=100,bbox_inches='tight')
