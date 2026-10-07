"""Per-range-band forward offset and free-width comparison (evening vs morning scenario3)."""
import os,sys,numpy as np
sys.argv=[sys.argv[0]]; exec(open('bev_fit.py').read().split("def score")[0])
def world_band(frames,ox,c0,c1):
    P,Vv=[],[]
    for p,a,g in frames:
        gg=g[:,c0:c1]; I=ii[:,c0:c1]; Jj=jj[:,c0:c1]
        bx=ox+RES*(I+.5); by=-20+RES*(Jj+.5); c,s=np.cos(a),np.sin(a); m=gg!=-1
        P.append(np.stack([p[0]+c*bx[m]-s*by[m],p[1]+s*bx[m]+c*by[m]],1)); Vv.append(gg[m]==0)
    return np.concatenate(P),np.concatenate(Vv)
for tag in os.environ.get('TAGS','180803').split(','):
    fr=load(tag,4); print('==',tag)
    for c0,c1 in ((0,32),(32,64),(64,96),(96,128)):
        sc=[]
        for ox in np.arange(-2,9.1,1.0):
            T=raster(*world_band(fr,ox,c0,c1)); tf={k for k,v in T.items() if v>.8}
            sc.append((len(tf&Rf)/max(1,len((tf|Rf)&set(T)&RS)),ox))
        b=max(sc); print(f'   cols {c0:3d}-{c1:3d} (body x {c0*RES:4.1f}-{c1*RES:4.1f} m @origin0): best forward offset {b[1]:+.1f} m IoU {b[0]:.3f} | IoU@0 {sc[2][0]:.3f}')
    # free lateral width per column band (median over frames, only rows containing free)
    w=[np.median([(g[:,c0:c1]==0).sum(0).mean()*RES for p,a,g in fr]) for c0,c1 in ((0,32),(32,64),(64,96),(96,128))]
    print('   median free width per column (m) by band:',[round(x,2) for x in w])
