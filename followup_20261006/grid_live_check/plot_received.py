from pathlib import Path
import numpy as np,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap,BoundaryNorm
from matplotlib.patches import Patch
p=Path(__file__).resolve().parent/'20261006_164936';a=np.load(p/'first_grid.npy');fig,axs=plt.subplots(1,2,figsize=(12,5))
for ax,origin,title in zip(axs,[2.95,0.],['As published: origin x = 2.95 m','As used by planner: origin overridden to 0 m']):
 ax.imshow(a,origin='lower',extent=[origin,origin+40,-20,20],interpolation='nearest',cmap=ListedColormap(['#55bad0','#e5a19a']),norm=BoundaryNorm([-0.5,50,100.5],2))
 for k in [1,2,3,4]:
  x=k*4.5;ax.axvline(x,color='#777777',lw=.8,ls=':');ax.text(x+.2,17,f'k{k}',fontsize=9)
 lam=np.linspace(-19.7,20.2,200);ang=np.radians(-1.01);x=13.498-lam*np.sin(ang);y=-.237+lam*np.cos(ang);ax.plot(x,y,'k--',lw=1.5,label='Logged k3 cross-section');ax.scatter([13.498],[-.237],c='black',s=25)
 ax.set_xlim(0,43);ax.set_ylim(-20,20);ax.set_aspect('equal');ax.set_title(title,fontsize=11);ax.set_xlabel('Vehicle forward x (m)');ax.set_ylabel('Vehicle lateral y (m)')
fig.legend(handles=[Patch(color='#55bad0',label='0: drivable'),Patch(color='#e5a19a',label='100: blocked')],loc='lower center',ncol=2);fig.suptitle('Live grid: road cells occupy only the first 35 columns; no unknown (-1) cells',fontsize=12);fig.tight_layout(rect=[0,.06,1,.95]);fig.savefig(p/'received_grid_explained.png',dpi=160)
