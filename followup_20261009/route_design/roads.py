# Python replica of matlab/knu_batch/map_gen_knu.m road rendering (boundary-profile widths, 1 m junction overlap, road27 connectors).
import numpy as np,scipy.io as sio,csv,collections
SRC='/home/imac/.codex/attachments/89097ea0-fc32-4321-b394-1fb41cf562b9/'
def cum(p): return np.concatenate([[0],np.cumsum(np.linalg.norm(np.diff(p[:,:2],axis=0),axis=1))])
def unit(v): return v/np.linalg.norm(v[:2])
def load():
    m=sio.loadmat(SRC+'processed_road_map.mat',simplify_cells=True)
    C=[np.asarray(c,float) for c in m['roadCenterlines']];W=np.asarray(m['roadWidths'],float);ids=list(m['roadIDs']);Q=list(m['roadQuality'])
    ws=collections.defaultdict(list)
    for r in csv.DictReader(open(SRC+'width_samples.csv',encoding='utf-8-sig')):
        if r['valid']!='0' and r['inlier']!='0':ws[int(r['road_id'])].append((float(r['station_m']),float(r['width_m'])))
    roads=[]
    for k,c in enumerate(C):
        out=c.copy()
        def touches(p):
            return any(np.min(np.linalg.norm(C[j][:,:2]-p[:2],axis=1))<=1.25 for j in range(len(C)) if j!=k)
        if touches(c[0]): out=np.vstack([c[0]-1.0*unit(c[1]-c[0]),out])
        if touches(c[-1]): out=np.vstack([out,c[-1]+1.0*unit(c[-1]-c[-2])])
        L=cum(c)[-1];rid=ids[k];s=sorted(dict(ws[rid]).items())
        if Q[k] in ('Reliable','Moderate') and len(s)>=6 and L>=2:
            st=np.array([a for a,_ in s]);wd=np.array([b for _,b in s]);n=min(20,max(2,int(np.ceil(L/12))));e=np.linspace(0,L,n+1);cen=(e[:-1]+e[1:])/2;lw=np.zeros(n)
            for i in range(n):
                ins=(st>=e[i])&(st<=e[i+1]);lw[i]=np.median(wd[ins]) if ins.any() else wd[np.argmin(abs(st-cen[i]))]
            lw=np.array([np.median(lw[max(0,i-1):i+2]) for i in range(n)]);lw=np.clip(lw,3,12)
            prof=(e,lw) if lw.max()-lw.min()>=0.1 else (np.array([0,L]),np.array([np.median(lw)]))
        else: prof=(np.array([0,L]),np.array([W[k]]))
        roads.append(dict(id=rid,quality=Q[k],center=out,raw=c,prof=prof,base=W[k]))
    # road27 connectors
    def idx(i): return ids.index(i)
    t=C[idx(27)]
    def mean_near(nb,pt): return np.mean([C[idx(i)][0] if np.linalg.norm(C[idx(i)][0,:2]-pt[:2])<=np.linalg.norm(C[idx(i)][-1,:2]-pt[:2]) else C[idx(i)][-1] for i in nb],axis=0)
    lj=mean_near([22,23,26],t[0]);rj=mean_near([28,29],t[-1]);c28=C[idx(28)]
    rat=unit(c28[1]-c28[0]) if np.linalg.norm(c28[0,:2]-rj[:2])<=np.linalg.norm(c28[-1,:2]-rj[:2]) else unit(c28[-2]-c28[-1])
    def cubic(p0,p3,t0,t3):
        ch=np.linalg.norm(p3[:2]-p0[:2]);cl=min(2,ch/3);p1=p0+cl*t0;p2=p3-cl*t3;tt=np.linspace(0,1,max(8,int(np.ceil(ch/.25))+1))[:,None]
        return (1-tt)**3*p0+3*(1-tt)**2*tt*p1+3*(1-tt)*tt**2*p2+tt**3*p3
    for nm,cc in [('C27L',cubic(lj,t[0],unit(t[0]-lj),unit(t[1]-t[0]))),('C27R',cubic(t[-1],rj,unit(t[-1]-t[-2]),rat))]:
        cc=np.vstack([cc[0]-unit(cc[1]-cc[0]),cc,cc[-1]+unit(cc[-1]-cc[-2])]);L=cum(cc)[-1]
        roads.append(dict(id=nm,quality='connector',center=cc,raw=cc,prof=(np.array([0,L]),np.array([W[idx(27)]])),base=W[idx(27)]))
    return roads
def width_at(road,s):
    e,w=road['prof'];return w[np.clip(np.searchsorted(e,s,side='right')-1,0,len(w)-1)]
def raster(roads,ext,res=0.25):
    X0,X1,Y0,Y1=ext;nx,ny=int((X1-X0)/res),int((Y1-Y0)/res);M=np.zeros((ny,nx),bool)
    xs=X0+(np.arange(nx)+.5)*res;ys=Y0+(np.arange(ny)+.5)*res
    for r in roads:
        c=r['center'][:,:2];s=cum(r['center'])-(np.linalg.norm(r['center'][0,:2]-r['raw'][0,:2]) if not np.allclose(r['center'][0],r['raw'][0]) else 0)
        for i in range(len(c)-1):
            a,b=c[i],c[i+1];hw=width_at(r,max(0,s[i]))/2;lo=np.minimum(a,b)-hw;hi=np.maximum(a,b)+hw
            ix=slice(max(0,int((lo[0]-X0)/res)),min(nx,int((hi[0]-X0)/res)+2));iy=slice(max(0,int((lo[1]-Y0)/res)),min(ny,int((hi[1]-Y0)/res)+2))
            GX,GY=np.meshgrid(xs[ix],ys[iy]);P=np.stack([GX,GY],-1);v=b-a;u=np.clip(((P-a)@v)/max(v@v,1e-12),0,1);d=np.linalg.norm(P-(a+u[...,None]*v),axis=-1)
            M[iy,ix]|=d<=hw
    return M
