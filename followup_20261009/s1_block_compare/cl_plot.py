import json,sys,numpy as np,matplotlib;matplotlib.use('Agg');import matplotlib.pyplot as plt
R=[json.loads(l) for l in open(sys.argv[1])];out=sys.argv[2]
sm=json.load(open('static_map.json'));M=np.fromfile('static_map.bin',dtype=np.int8).reshape(sm['height'],sm['width'])
fig,ax=plt.subplots(3,1,figsize=(13,14))
ax[0].imshow(M,origin='lower',cmap='gray_r',extent=(sm['origin'][0],sm['origin'][0]+sm['width']*sm['res'],sm['origin'][1],sm['origin'][1]+sm['height']*sm['res']))
for r in R:
    S=r['states'];x=np.array([s['x'] for s in S]);y=np.array([s['y'] for s in S]);c=np.degrees([s['cmd'] for s in S])
    lab=f"{r['config']['name']} {r['start']['name']} ph{r['config']['phase']} {r['outcome']}"
    ax[0].plot(x,y,lw=1,label=lab);ax[1].plot(x,c,lw=1);P=r['plans'];ax[2].plot([p['x'] for p in P],[p['B'] for p in P],lw=1,drawstyle='steps-post')
ax[0].set_xlim(100,170);ax[0].set_ylim(-65,15);ax[0].set_aspect('equal');ax[0].legend(fontsize=6)
ax[1].set_xlim(100,170);ax[1].set_ylabel('cmd deg (vs x)');ax[2].set_xlim(100,170);ax[2].set_ylabel('B (vs x)')
for a in ax: a.grid(alpha=.3)
fig.savefig(out,dpi=80,bbox_inches='tight')
