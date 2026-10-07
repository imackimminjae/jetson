"""Fit evening BEV geometry (x-origin, y-origin, cell scale, yaw offset) against the morning scenario3 reference map."""
import os,sys,numpy as np
sys.argv=[sys.argv[0]]; exec(open('bev_alignment.py').read().split("ref = []")[0])
def world(frames,ox,oy,res,dyaw):
    P,Vv=[],[]
    for p,a,g in frames:
        bx=ox+res*(ii+.5); by=oy+res*(jj+.5); c,s=np.cos(a+dyaw),np.sin(a+dyaw)
        m=g!=-1; P.append(np.stack([p[0]+c*bx[m]-s*by[m],p[1]+s*bx[m]+c*by[m]],1)); Vv.append(g[m]==0)
    return np.concatenate(P),np.concatenate(Vv)
ref=[]
for t in ['032453','032545','045314']: ref+=load(t,3)
R=raster(*world(ref,0,-20,RES,0)); Rf={k for k,v in R.items() if v>.8}; Ro={k for k,v in R.items() if v<.2}; RS=set(R)
def score(fr,*a):
    T=raster(*world(fr,*a)); tf={k for k,v in T.items() if v>.8}
    return len(tf&Rf)/max(1,len((tf|Rf)&set(T)&RS))
for tag in os.environ.get('TAGS','180803').split(','):
    fr=load(tag,4); best=(0,)
    for res in (0.28,0.3125,0.34,0.36):
        h=res*128/2
        for ox in (0,1.5,3,4.5,6):
            for oy in (-h-1.0,-h,-h+1.0):
                for dy in (-3,0,3):
                    s=score(fr,ox,oy,res,np.radians(dy))
                    if s>best[0]: best=(s,ox,oy,res,dy)
    print(tag,'best IoU=%.3f x0=%.1f y0=%.2f res=%.4f yaw_off=%+d deg'%best,flush=True)
    s,ox,oy,res,dy=best
    for d in (-1,1):
        print('   refine ox',ox+d*0.75,'%.3f'%score(fr,ox+d*.75,oy,res,np.radians(dy)),' oy',oy+d*0.5,'%.3f'%score(fr,ox,oy+d*.5,res,np.radians(dy)),' yaw',dy+d,'%.3f'%score(fr,ox,oy,res,np.radians(dy+d)))
