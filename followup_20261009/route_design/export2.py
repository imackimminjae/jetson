import sys,pickle,json,math,numpy as np,os
sys.path.insert(0,'.')
from band import radius,sample,DIN
from graph2 import edges as EDGES
O=pickle.load(open('pipeline_out.pkl','rb'));R=O['routes']
order=[3,2,10,12,0,9,1,4,8]   # pipeline indices; old S7,S8,S14 removed on user request 2026-10-09
region=['서쪽','서쪽','서쪽 회전교차로','서쪽 회전교차로','서쪽→중앙','중앙 남쪽','중앙→북동','북동→중앙','북동 회전교차로']
def din(P): return sample(DIN,P)
def chord_ok(a,b,m):
    n=max(2,int(np.linalg.norm(b-a)/0.5));return din(a+np.outer(np.linspace(0,1,n),b-a)).min()>=m
def dp(P,tol):
    if len(P)<3: return [0,len(P)-1]
    a,b=P[0],P[-1];v=b-a;n=np.linalg.norm(v);d=np.abs(np.cross(v,P-a))/n if n>1e-9 else np.linalg.norm(P-a,axis=1);i=int(np.argmax(d))
    if d[i]<=tol: return [0,len(P)-1]
    l=dp(P[:i+1],tol);r=dp(P[i:],tol);return l[:-1]+[x+i for x in r]
def waypoints(Z):
    wp=[Z[i] for i in dp(Z,0.5)];out=[wp[0]]
    for a,b in zip(wp[:-1],wp[1:]):
        n=int(np.ceil(np.linalg.norm(b-a)/30.0));out+=[a+(b-a)*j/n for j in range(1,n+1)]
    wp=out;changed=True
    while changed:
        changed=False
        for i in range(1,len(wp)-1):
            if min(np.linalg.norm(wp[i]-wp[i-1]),np.linalg.norm(wp[i+1]-wp[i]))<10 and np.linalg.norm(wp[i+1]-wp[i-1])<=34 and chord_ok(wp[i-1],wp[i+1],0.7):
                del wp[i];changed=True;break
    # repair chords that leave the road: insert the farthest dense point between the two waypoints
    k=0
    while k<len(wp)-1:
        if not chord_ok(wp[k],wp[k+1],0.6):
            ia=int(np.argmin(np.linalg.norm(Z-wp[k],axis=1)));ib=int(np.argmin(np.linalg.norm(Z-wp[k+1],axis=1)));seg=Z[ia+1:ib]
            if len(seg):
                v=wp[k+1]-wp[k];d=np.abs(np.cross(v,seg-wp[k]))/max(np.linalg.norm(v),1e-9);wp.insert(k+1,seg[int(np.argmax(d))]);continue
        k+=1
    return np.round(np.array(wp),3)
os.makedirs('out12',exist_ok=True);starts=[];anchors=[];mat=[];doc=[];final=[]
for slot,(i,reg) in enumerate(zip(order,region),start=4):
    r=R[i];t=r['trail'];Z=r['dense'];W=waypoints(Z);name=f'scenario{slot}'
    with open(f'out12/{name}.csv','w') as f:
        f.write('x,y,z\n');[f.write(f'{x:.3f},{y:.3f},0.0\n') for x,y in W]
    first=[float(W[0,0]),float(W[0,1]),0.0];d5=Z[10]-Z[0];yaw=math.atan2(d5[1],d5[0]);second=[round(first[0]+5*math.cos(yaw),9),round(first[1]+5*math.sin(yaw),9),0.0];yaw=math.atan2(second[1]-first[1],second[0]-first[0])
    rid=[EDGES[h['e']]['rid'] for h in t];rid=[x if isinstance(x,str) else int(x) for x in rid];roads=[x for k,x in enumerate(rid) if k==0 or x!=rid[k-1]]
    junc=[]
    for h1,h2 in zip(t[:-1],t[1:]):
        ang=math.degrees(math.atan2(np.cross(h1['t1'],h2['t0']),h1['t1']@h2['t0']));junc.append(f"n{h1['b']}:{'좌' if ang>25 else '우' if ang<-25 else '직'}{abs(ang):.0f}°")
    sp=np.linalg.norm(np.diff(W,axis=0),axis=1);chords=all(chord_ok(W[k],W[k+1],0.6) for k in range(len(W)-1))
    L=float(np.sum(np.linalg.norm(np.diff(Z,axis=0),axis=1)))
    starts.append(dict(scenario=name,position=first,path_first=first,heading_rad=yaw,ros_heading_rad=yaw,original_position=first,original_point_on_road=True,original_body_on_road=True,shift_m=0,corrected=False,body_on_road=True,clearance_margin_m=round(float(din(Z[:1])[0])-0.9,2),section_number=slot,generator='followup_20261009/route_design'))
    anchors.append(f'  if (route_name == "{name}") {{\n    return RouteAnchor{{{{{first[0]:.9f}, {first[1]:.9f}, 0.000000000}}, {{{second[0]:.9f}, {second[1]:.9f}, 0.000000000}}}};\n  }}')
    mat.append(f'    case "{name}"\n        sectionName = "{reg}";\n        waypoints = [ ...\n'+''.join(f'            {x:.3f} {y:.3f} 0\n' for x,y in W)+'            ];')
    desc=f"{reg}: n{t[0]['a']}{'(막다른 끝)' if t[0]['a'] in __import__('gen').DEAD else ''} → n{t[-1]['b']}{'(막다른 끝)' if t[-1]['b'] in __import__('gen').DEAD else ''}"
    doc.append(dict(name=name,desc=desc,length_m=round(L,1),waypoints=len(W),roads=roads,junctions=junc,min_radius_m=round(r['rmin'],1),min_center_clearance_m=round(r['clear'],2),wp_spacing_m=[round(float(sp.min()),1),round(float(sp.max()),1)],chords_on_road=chords))
    final.append(dict(name=name,desc=desc,dense=Z,wp=W,len=L,din_min=r['clear'],rmin=r['rmin']))
json.dump(starts,open('out12/start_entries.json','w'),indent=1,ensure_ascii=False)
open('out12/px4_anchor_snippet.cpp','w').write('\n'.join(anchors)+'\n');open('out12/matlab_cases_snippet.m','w').write('\n'.join(mat)+'\n')
prof=dict(version=3,profile='centerline12',routes=['scenario1','scenario2','scenario3']+[d['name'] for d in doc],preserved_routes=['scenario1','scenario2','scenario3'],route_count=3+len(doc),length_m=round(sum(d['length_m'] for d in doc),1),
  policy='Scenarios 1-3 unchanged. scenario4-12 regenerated on processed_road_map.mat centerlines: every junction turn <=95 deg, min path radius >=9.5 m, vehicle-center clearance >=0.9 m (followup_20261009/route_design).')
json.dump(prof,open('out12/knu_batch_profile.json','w'),indent=1);json.dump(doc,open('out12/routes_summary.json','w'),indent=1,ensure_ascii=False)
pickle.dump(final,open('routes_final2.pkl','wb'))
for d in doc: print(f"{d['name']:10s} {d['desc']:32s} {d['length_m']:6.1f} m wp{d['waypoints']:2d} 간격{d['wp_spacing_m']} 반경≥{d['min_radius_m']} 여유≥{d['min_center_clearance_m']} 직선OK={d['chords_on_road']} | {' '.join(d['junctions'])}")
print('new routes total',prof['length_m'],'m')
