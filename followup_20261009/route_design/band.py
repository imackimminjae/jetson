import numpy as np,json
from scipy import ndimage
M=np.load('road_mask.npy');ext=json.load(open('road_mask.json'))['ext'];DIN=ndimage.distance_transform_edt(M)*0.25
GY,GX=np.gradient(ndimage.gaussian_filter(DIN,1.0))
def sample(A,P):
    fx=(P[:,0]-ext[0])/.25-.5;fy=(P[:,1]-ext[2])/.25-.5
    return ndimage.map_coordinates(A,[fy,fx],order=1,mode='nearest')
def resample(P,step=0.5):
    s=np.concatenate([[0],np.cumsum(np.linalg.norm(np.diff(P,axis=0),axis=1))]);k=np.r_[True,np.diff(s)>1e-6];P,s=P[k],s[k]
    t=np.linspace(0,s[-1],max(3,int(s[-1]/step)+1));return np.c_[np.interp(t,s,P[:,0]),np.interp(t,s,P[:,1])]
def radius(D,win=10):
    a=D[:-2*win];b=D[win:-win];c=D[2*win:];ab=np.linalg.norm(b-a,axis=1);bc=np.linalg.norm(c-b,axis=1);ca=np.linalg.norm(a-c,axis=1);area=np.abs(np.cross(b-a,c-a))/2
    return np.where(area>1e-9,ab*bc*ca/(4*np.maximum(area,1e-12)),np.inf)
def band(P,clear=1.1,rmin=11.0,iters=4000,fix=6):
    P=resample(P)
    for it in range(iters):
        L=np.zeros_like(P);L[1:-1]=P[:-2]+P[2:]-2*P[1:-1];L[:fix]=0;L[-fix:]=0
        P=P+0.35*L
        d=sample(DIN,P);bad=d<clear
        if bad.any():
            g=np.c_[sample(GX,P[bad]),sample(GY,P[bad])]/.25;n=np.linalg.norm(g,axis=1,keepdims=True);P[bad]+=g/np.maximum(n,1e-9)*np.minimum(clear-d[bad],0.3)[:,None]
        if it%50==49:
            P=resample(P)
            if len(P)>2*10+1 and radius(P).min()>=rmin and sample(DIN,P).min()>=clear-0.05: break
    P=resample(P);return P,float(radius(P).min()) if len(P)>21 else np.inf,float(sample(DIN,P).min())

def bend(P,clear=1.1,rmin=11.0,iters=20000,fix=8,alpha=0.06):
    """Minimize bending energy (4th difference) with a road-clearance constraint; endpoints and end tangents fixed."""
    P=resample(P);best=(None,-1)
    for it in range(iters):
        B=np.zeros_like(P);B[2:-2]=P[:-4]-4*P[1:-3]+6*P[2:-2]-4*P[3:-1]+P[4:];B[:fix]=0;B[-fix:]=0
        P=P-alpha*B
        d=sample(DIN,P);bad=d<clear
        if bad.any():
            g=np.c_[sample(GX,P[bad]),sample(GY,P[bad])]/.25;n=np.linalg.norm(g,axis=1,keepdims=True);P[bad]+=g/np.maximum(n,1e-9)*np.minimum(clear-d[bad],0.2)[:,None]
        if it%100==99:
            P=resample(P);rm=radius(P).min();cl=sample(DIN,P).min()
            if cl>=clear-0.1 and rm>best[1]: best=(P.copy(),rm)
            if rm>=rmin and cl>=clear-0.05: break
    P=best[0] if best[0] is not None else resample(P)
    return P,float(radius(P).min()),float(sample(DIN,P).min())

from scipy.optimize import minimize
def optimize(P,clear=1.1,step=2.0,mu=200.0,w_len=0.02):
    """Racing-line style: minimize sum |second difference|^2 + penalty for clearance violation; first/last 2 points fixed."""
    Q=resample(P,step);n=len(Q);fixed=np.r_[0,1,n-2,n-1];free=np.setdiff1d(np.arange(n),fixed)
    def unpack(x):
        Z=Q.copy();Z[free]=x.reshape(-1,2);return Z
    def f(x):
        Z=unpack(x);D2=Z[:-2]-2*Z[1:-1]+Z[2:];e=np.sum(D2**2)
        D1=np.diff(Z,axis=0);e+=w_len*np.sum((np.linalg.norm(D1,axis=1)-step)**2)
        d=sample(DIN,Z);v=np.maximum(0,clear-d);e+=mu*np.sum(v**2)
        g=np.zeros_like(Z);g[:-2]+=2*D2;g[1:-1]+=-4*D2;g[2:]+=2*D2
        nl=np.linalg.norm(D1,axis=1,keepdims=True);u=D1/np.maximum(nl,1e-9);gl=2*w_len*(nl-step)*u;g[1:]+=gl;g[:-1]-=gl
        gd=np.c_[sample(GX,Z),sample(GY,Z)]/.25;g+=(-2*mu*v)[:,None]*gd
        return e,g[free].ravel()
    x0=Q[free].ravel()
    for m in (mu,mu*10):
        mu=m;res=minimize(f,x0,jac=True,method='L-BFGS-B',options=dict(maxiter=4000));x0=res.x
    Z=resample(unpack(x0),0.5);return Z,float(radius(Z).min()),float(sample(DIN,Z).min())
