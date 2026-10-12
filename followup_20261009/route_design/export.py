import sys,pickle,json,math,numpy as np,os
sys.path.insert(0,'.')
from gen import H,turn
g=pickle.load(open('graph.pkl','rb'));edges=g['edges']
R=pickle.load(open('routes.pkl','rb'))
# order by region: West, Center, NE
order=[3,2,6,10,11,0,4,8,1,9,5,7]   # indices into R (R4,R3,R7,R11,R12,R1,R5,R9,R2,R10,R6,R8)
region={3:'서쪽',2:'서쪽',6:'서쪽',10:'서쪽',11:'서쪽',0:'서쪽→중앙',4:'중앙',8:'중앙 남쪽',1:'중앙→북동',9:'북동→중앙',5:'북동 회전교차로',7:'북동'}
HB={(h['e'],h['a'],h['b']):h for h in H}
os.makedirs('out',exist_ok=True)
starts=[];anchors=[];mat=[];doc=[];final=[]
for slot,i in enumerate(order,start=4):
    r=R[i];name=f'scenario{slot}';W=np.round(r['wp'],3);D=r['dense']
    with open(f'out/{name}.csv','w') as f:
        f.write('x,y,z\n');[f.write(f'{x:.3f},{y:.3f},0.0\n') for x,y in W]
    s5=D[min(10,len(D)-1)];yaw=math.atan2(s5[1]-W[0,1],s5[0]-W[0,0]);second=[round(W[0,0]+5*math.cos(yaw),9),round(W[0,1]+5*math.sin(yaw),9),0.0]
    first=[float(W[0,0]),float(W[0,1]),0.0];yaw=math.atan2(second[1]-first[1],second[0]-first[0])
    rid=[edges[e]['rid'] if isinstance(edges[e]['rid'],str) else int(edges[e]['rid']) for e,_,_ in r['trail']];roads=[x for k,x in enumerate(rid) if k==0 or x!=rid[k-1]]
    junc=[]
    for (e1,a1,b1),(e2,a2,b2) in zip(r['trail'][:-1],r['trail'][1:]):
        h1,h2=HB[(e1,a1,b1)],HB[(e2,a2,b2)];ang=math.degrees(math.atan2(np.cross(h1['t1'],h2['t0']),h1['t1']@h2['t0']))
        junc.append(f"n{b1}:{'좌' if ang>25 else '우' if ang<-25 else '직'}{abs(ang):.0f}°")
    starts.append(dict(scenario=name,position=first,path_first=first,heading_rad=yaw,ros_heading_rad=yaw,original_position=first,original_point_on_road=True,original_body_on_road=True,shift_m=0,corrected=False,body_on_road=True,clearance_margin_m=round(r['din_min'],2),section_number=slot,route_generator='followup_20261009/route_design',centerline_based=True))
    anchors.append(f'  if (route_name == "{name}") {{\n    return RouteAnchor{{{{{first[0]:.9f}, {first[1]:.9f}, 0.000000000}}, {{{second[0]:.9f}, {second[1]:.9f}, 0.000000000}}}};\n  }}')
    mat.append(f'    case "{name}"\n        waypoints = [ ...\n'+''.join(f'            {x:.3f} {y:.3f} 0\n' for x,y in W)+'            ];')
    desc=f"{region[i]}: n{r['nodes'][0]}{'(막다른 끝)' if r['start_dead'] else ''} → n{r['nodes'][-1]}{'(막다른 끝)' if r['end_dead'] else ''}"
    doc.append(dict(name=name,desc=desc,length_m=round(r['len'],1),waypoints=len(W),roads=roads,junctions=junc,min_clearance_m=round(r['din_min'],2),spacing_m=[round(x,1) for x in r['spacing']]))
    final.append(dict(r,name=name,desc=desc))
json.dump(starts,open('out/start_entries.json','w'),indent=1,ensure_ascii=False)
open('out/px4_anchor_snippet.cpp','w').write('\n'.join(anchors)+'\n')
open('out/matlab_cases_snippet.m','w').write('\n'.join(mat)+'\n')
prof=dict(version=3,profile='centerline15',routes=['scenario1','scenario2','scenario3']+[d['name'] for d in doc],preserved_routes=['scenario1','scenario2','scenario3'],route_count=3+len(doc),
          length_m=round(sum(d['length_m'] for d in doc),1),policy='Scenarios 1-3 unchanged; scenario4-15 regenerated on processed_road_map.mat centerlines to cover every road edge (route_design 2026-10-09).')
json.dump(prof,open('out/knu_batch_profile.json','w'),indent=1)
json.dump(doc,open('out/routes_summary.json','w'),indent=1,ensure_ascii=False)
pickle.dump(final,open('routes_final.pkl','wb'))
for d in doc: print(d['name'],d['desc'],f"{d['length_m']} m wp{d['waypoints']} roads {d['roads']} | {' '.join(d['junctions'])}")
print('total new length',prof['length_m'])
