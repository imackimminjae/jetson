from pathlib import Path
from datetime import datetime
from zoneinfo import ZoneInfo
import json,numpy as np
here=Path(__file__).resolve().parent
clock=lambda t:datetime.fromtimestamp(t,ZoneInfo('Asia/Seoul')).strftime('%H:%M:%S.%f')[:-3]
def heading(poly):
 a=np.array(poly);d=a[-1]-a[-2];return float(np.degrees(np.arctan2(d[1],d[0])))
def classify(row):
 if not row['valid']:return 'failed'
 q=np.array(row['planned'][-1])-np.array([121.,-7.])
 if np.linalg.norm(q)<8 or q[1]<-4:return 'before_fork'
 return 'E' if np.degrees(np.arctan2(q[1],q[0]))<42 else 'N'
def progress(route,xy):
 a=np.array(route);v=np.diff(a,axis=0);l=np.linalg.norm(v,axis=1);d=xy-a[:-1];u=np.clip(np.sum(d*v,axis=1)/l**2,0,1);q=a[:-1]+u[:,None]*v;i=np.argmin(np.sum((q-xy)**2,axis=1));return float(np.r_[0,np.cumsum(l)][i]+u[i]*l[i])
def p95(a):return float(np.quantile(a,.95)) if a else None
summary={};detailed={}
for tag in ['050536','050627','014143','014303','045314','044738']:
 inp=json.loads((here/f'replay_{tag}.json').read_text());cycles=inp['cycles'];prog=[progress(inp['route'],np.array([c['x'],c['y']])) for c in cycles]
 for cfg in ['recorded','current']:
  results=json.loads((here/f'result_{tag}_{cfg}.json').read_text());stats={};detail=[]
  for key,rows in results.items():
   comparisons=[];equalvalid=0;near=[];nearj=[];errors=[]
   for i,(c,r) in enumerate(zip(cycles,rows)):
    equalvalid+=int(c['rec_valid']==r['valid'])
    if c['rec_valid'] and r['valid'] and len(c['rec_planned'])==len(r['planned']):
     err=float(np.max(np.linalg.norm(np.array(c['rec_planned'])-np.array(r['planned']),axis=1)));comparisons.append(err);errors.append([clock(c['t']),err])
    if r['valid'] and len(r['planned'])>2:
     p=np.array(r['planned']);ref=np.array(r['reference']);delta=p[1]-p[0];rt=ref[1]-ref[0];ang=abs(np.degrees(np.arctan2(np.cross(rt,delta),rt@delta)))
     near.append(ang)
     if tag.startswith('050') and 118<=prog[i]<=150:nearj.append(ang)
   stats[key]={'failures':sum(not r['valid'] for r in rows),'cycles':len(rows),'baseline_record_valid_match':equalvalid,'record_geometry_comparisons':len(comparisons),'record_geometry_match_1mm':sum(v<.001 for v in comparisons),'record_geometry_error_p95_m':p95(comparisons),'record_geometry_error_max_m':max(comparisons) if comparisons else None,'near_heading_correction_p95_deg':p95(near),'J2_near_heading_correction_p95_deg':p95(nearj),'failure_times':[clock(r['t']) for r in rows if not r['valid']]}
   if cfg=='recorded' and key=='1_0.000000':stats[key]['geometry_mismatch_gt_1mm']=[a for a in errors if a[1]>.001]
  if tag.startswith('050'):
   for i,c in enumerate(cycles):
    if not 115<=prog[i]<=151:continue
    row={'time':clock(c['t']),'t':c['t'],'progress':prog[i],'xy':[c['x'],c['y']]}
    for key,rows in results.items():
     r=rows[i];row[key]={'choice':classify(r),'B':r['B'],'near_heading_deg':float(np.degrees(np.arctan2(r['planned'][1][1]-c['y'],r['planned'][1][0]-c['x']))) if r['valid'] else None,'end':r.get('planned',[[None,None]])[-1],'selected':r['selected'],'turn_steps':r['turn_steps'],'status':r['status']}
    detail.append(row)
  summary[tag+'/'+cfg]=stats;detailed[tag+'/'+cfg]=detail
  print(tag,cfg, {k:(v['failures'],v['record_geometry_match_1mm'],v['record_geometry_comparisons']) for k,v in stats.items()})
(here/'summary.json').write_text(json.dumps(summary,indent=2)+'\n');(here/'J2_decisions.json').write_text(json.dumps(detailed,indent=2)+'\n')
for tag in ['050536','050627']:
 for cfg in ['recorded','current']:
  print(tag,cfg)
  for row in detailed[tag+'/'+cfg]:
   print(row['time'],round(row['progress'],1),{k:(row[k]['choice'],round(row[k]['B'],1),[round(v,1) if v is not None else None for v in row[k]['end']]) for k in ['0_0.000000','0_12.000000','1_0.000000','1_12.000000']})
