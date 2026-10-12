# Route generator: trails on the junction graph that cover every road edge, then centerline polylines -> sparse waypoints.
import sys,pickle,json,csv,math,numpy as np,collections
sys.path.insert(0,'.')
from scipy import ndimage
from graph2 import piece
g=pickle.load(open('graph.pkl','rb'));edges,nodes=g['edges'],g['nodes']
M=np.load('road_mask.npy');ext=json.load(open('road_mask.json'))['ext'];DIN=ndimage.distance_transform_edt(M)*0.25
def din(P):
    ix=np.clip(((P[:,0]-ext[0])/.25).astype(int),0,M.shape[1]-1);iy=np.clip(((P[:,1]-ext[2])/.25).astype(int),0,M.shape[0]-1);return DIN[iy,ix]
USE=[e for e in edges if not (e['n0']==e['n1'] and e['L']<10)]
deg=collections.Counter()
for e in USE: deg[e['n0']]+=1;deg[e['n1']]+=1
DEAD={n for n,d in deg.items() if d==1}
def tang(P,end):
    if end: a,b=P[max(0,len(P)-13)],P[-1]
    else: a,b=P[0],P[min(len(P)-1,12)]
    v=b-a;return v/max(np.linalg.norm(v),1e-9)
# directed half-edges
H=[]
for e in USE:
    for fwd in (True,False):
        P=piece(e,fwd);H.append(dict(e=e['i'],a=e['n0'] if fwd else e['n1'],b=e['n1'] if fwd else e['n0'],P=P,L=e['L'],t0=tang(P,False),t1=tang(P,True)))
OUT=collections.defaultdict(list)
for h in H: OUT[h['a']].append(h)
def turn(h1,h2):
    c=float(np.clip(h1['t1']@h2['t0'],-1,1));return math.degrees(math.acos(c))
MAXTURN=120;MAXLEN=350
cands=[]
def dfs(trail,length,used,visited):
    last=trail[-1];ends_dead=last['b'] in DEAD
    if length>=110 or ends_dead: cands.append(list(trail))
    if ends_dead: return
    for h in OUT[last['b']]:
        if length+h['L']>MAXLEN+(30 if h['b'] in DEAD else 0): continue
        if h['e'] in used or h['b'] in visited and not (h['b']==last['b']): continue
        if turn(last,h)>MAXTURN: continue
        trail.append(h);used.add(h['e']);visited.add(h['b'])
        dfs(trail,length+h['L'],used,visited)
        trail.pop();used.discard(h['e']);visited.discard(h['b'])
for h in H:
    if h['a'] in DEAD or h['L']>=60:   # start at a dead end, or inside a long road
        dfs([h],h['L'],{h['e']},{h['a'],h['b']})
print('candidates',len(cands),'dead ends',sorted(DEAD))
pickle.dump(dict(cands=cands,USE=USE,DEAD=DEAD),open('cands.pkl','wb'))
