from pathlib import Path
import sqlite3,json,math
from collections import Counter
from rclpy.serialization import deserialize_message
from std_msgs.msg import String
root=Path('/home/imac/ros2_ws');out=[]
for name in ['drive_debug_20260923_232054_light','drive_debug_20260924_001550_light']:
 db=sqlite3.connect(f'file:{root/name}/bag/bag_0.db3?mode=ro',uri=True)
 tid=db.execute("select id from topics where name='/debug/upper_branch_event'").fetchone()[0]
 events=[json.loads(deserialize_message(blob,String).data) for blob, in db.execute('select data from messages where topic_id=? order by timestamp',(tid,))];db.close()
 counts=Counter();selected_counts=Counter();rows=[];point_checks=[];examples={};cycle_modes=[];widened=0;narrowed=0
 for e in events:
  modes=Counter()
  for row in e['intervals_world']:
   k,j,*_=row;raw,processed=row[6:8];expected_on=.4*raw;expected_off=raw-2*min(2.,.45*raw)
   on=abs(processed-expected_on)<1e-5;off=abs(processed-expected_off)<1e-5
   mode='ambiguous' if on and off else 'ON' if on else 'OFF' if off else 'mismatch'
   counts[mode]+=1;modes[mode]+=1
   if mode=='OFF':
    widened+=int(processed>expected_on+1e-5);narrowed+=int(processed<expected_on-1e-5)
   selection=e.get('selected_corridors',[])
   selected=len(selection)>k and selection[k]==j
   if selected:selected_counts[mode]+=1
   detail=dict(t=e['t_start'],seq=e['plan_seq'],k=k,candidate=j,raw=raw,processed=processed,mode=mode,source=row[8],selected=selected,valid=e['valid_plan'])
   rows.append(detail)
   if mode not in examples or (mode=='OFF' and raw>examples[mode]['raw']):examples[mode]=detail
   planned=e.get('planned_world',[])
   if selected and e['valid_plan'] and len(planned)>k:
    a,b=row[2:4],row[4:6];px,py=planned[k];dx,dy=b[0]-a[0],b[1]-a[1];length=math.hypot(dx,dy);along=((px-a[0])*dx+(py-a[1])*dy)/length
    point_checks.append(max(0.,-along,along-length))
  cycle_modes.append(dict(seq=e['plan_seq'],t=e['t_start'],modes=dict(modes),valid=e['valid_plan']))
 summary=dict(bag=name,events=len(events),counts=dict(counts),selected_counts=dict(selected_counts),off_wider_than_on=widened,off_narrower_than_on=narrowed,examples=examples,selected_point_checks=len(point_checks),max_processed_boundary_violation_m=max(point_checks,default=0),cycles_with_on=sum(x['modes'].get('ON',0)>0 for x in cycle_modes),cycles_with_off=sum(x['modes'].get('OFF',0)>0 for x in cycle_modes))
 out.append(dict(summary=summary,rows=rows,cycles=cycle_modes));print(json.dumps(summary,indent=2))
(root/'soft_ratio_analysis_20260924/recorded_interval_results.json').write_text(json.dumps(out,indent=2))
