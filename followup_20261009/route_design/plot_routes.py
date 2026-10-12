import sys,pickle,json,numpy as np,matplotlib;matplotlib.use('Agg');import matplotlib.pyplot as plt
from matplotlib import font_manager
font_manager.fontManager.addfont('/usr/share/fonts/truetype/nanum/NanumGothic.ttf');plt.rcParams.update({'font.family':'NanumGothic','axes.unicode_minus':False})
R=pickle.load(open(sys.argv[1] if len(sys.argv)>1 else 'routes.pkl','rb'));out=sys.argv[2] if len(sys.argv)>2 else 'routes_review.png'
M=np.load('road_mask.npy');ext=json.load(open('road_mask.json'))['ext']
n=len(R);cols=4;rows=int(np.ceil(n/cols))
fig,axs=plt.subplots(rows,cols,figsize=(6*cols,6*rows))
for ax,r in zip(axs.flat,R):
    ax.imshow(M,origin='lower',extent=(ext[0],ext[1],ext[2],ext[3]),cmap='Greys',alpha=.35)
    D=r['dense'];W=r['wp'];ax.plot(D[:,0],D[:,1],'-',c='tab:blue',lw=1.2);ax.plot(W[:,0],W[:,1],'o-',c='tab:red',ms=4,lw=.8)
    ax.annotate('',xy=D[min(30,len(D)-1)],xytext=D[0],arrowprops=dict(arrowstyle='-|>',color='green',lw=2.5));ax.plot(*D[0],'gs',ms=9);ax.plot(*D[-1],'k*',ms=12)
    pad=25;lo=D.min(0)-pad;hi=D.max(0)+pad;c=(lo+hi)/2;h=max(hi-lo)/2;ax.set_xlim(c[0]-h,c[0]+h);ax.set_ylim(c[1]-h,c[1]+h);ax.set_aspect('equal')
    ax.set_title(f"{(r['name'] if 'name' in r else 'R%d'%(r['k']+1))}  {r['len']:.0f} m · wp {len(W)} · 여유 {r['din_min']:.1f} m",fontsize=11);ax.grid(alpha=.25)
for ax in list(axs.flat)[n:]: ax.axis('off')
fig.savefig(out,dpi=55,bbox_inches='tight')
