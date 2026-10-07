"""Read-only complete OccupancyGrid audit of available drive_debug bags.
Known means >=0 in the message, not verified camera accuracy or drivable road.
"""
from pathlib import Path
import sqlite3,json,hashlib,csv,math
from collections import Counter
from datetime import datetime
from zoneinfo import ZoneInfo
import numpy as np
from rclpy.serialization import deserialize_message
from nav_msgs.msg import OccupancyGrid
H=Path(__file__).resolve().parent;R=H.parents[1];KST=ZoneInfo('Asia/Seoul')
clock=lambda ns:datetime.fromtimestamp(ns/1e9,KST).isoformat(timespec='milliseconds')
primary={'014143','014303','044738','045314','050536','050627','055105'}
patterns={};summaries=[];sources=[];runs=[];exceptions=[]
fields=['bag','db','message_id','timestamp_ns','time_kst','header_stamp_ns','pattern_id','known_cells','unknown_cells','free_cells','nonzero_known_cells','x_min_m','x_max_m','y_min_m','y_max_m','bbox_holes','centerline_known_contiguous_end_m','free_x_max_m']
with (H/'all_frames.csv').open('w',newline='') as f:
 writer=csv.DictWriter(f,fieldnames=fields);writer.writeheader()
 for folder in sorted(R.glob('drive_debug_*')):
  dbs=sorted((folder/'bag').glob('*.db3'))
  if not dbs:continue
  bname=folder.name;stats=Counter();freeends=[];centerends=[];stampzeros=0;frames=0;first=last=None;previous=None;interval=None
  for p in dbs:
   before=p.stat();stream=hashlib.sha256();con=sqlite3.connect('file:'+str(p)+'?mode=ro',uri=True);con.execute('BEGIN')
   topics=con.execute("SELECT id,name FROM topics WHERE name='/bev/occupancy_grid' AND type='nav_msgs/msg/OccupancyGrid'").fetchall()
   for topic_id,topic in topics:
    expected=con.execute('SELECT count(*) FROM messages WHERE topic_id=?',(topic_id,)).fetchone()[0];read=0
    for mid,ns,blob in con.execute('SELECT id,timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp,id',(topic_id,)):
     stream.update(str(mid).encode()+b':'+str(ns).encode()+b':'+blob)
     m=deserialize_message(blob,OccupancyGrid);w,ht=int(m.info.width),int(m.info.height);res=float(m.info.resolution);o=m.info.origin.position;q=m.info.origin.orientation
     g=np.asarray(m.data,dtype=np.int8).reshape(ht,w);known=g>=0;n=int(known.sum());unknown=g.size-n;free=g==0;zero=int(free.sum());stamp=m.header.stamp.sec*1000000000+m.header.stamp.nanosec
     meta=dict(width=w,height=ht,resolution=res,frame=m.header.frame_id,origin=[o.x,o.y,o.z],origin_q=[q.x,q.y,q.z,q.w])
     # Quaternion here is OccupancyGrid origin orientation, not project's scalar vehicle yaw.
     axis_aligned=abs(q.x)<1e-12 and abs(q.y)<1e-12 and abs(q.z)<1e-12 and (abs(abs(q.w)-1)<1e-12 or abs(q.w)<1e-12)
     yy,xx=np.where(known);bbox=[int(xx.min()),int(xx.max()),int(yy.min()),int(yy.max())] if n else None
     if n:
      xmin=o.x+bbox[0]*res;xmax=o.x+(bbox[1]+1)*res;ymin=o.y+bbox[2]*res;ymax=o.y+(bbox[3]+1)*res;holes=(bbox[1]-bbox[0]+1)*(bbox[3]-bbox[2]+1)-n
     else:xmin=xmax=ymin=ymax=None;holes=0
     maskhash=hashlib.sha256(np.packbits(known).tobytes()).hexdigest()
     pattern=dict(metadata=meta,axis_aligned=axis_aligned,known_bbox_indices=bbox,known_cells=n,unknown_cells=unknown,bbox_holes=int(holes),known_extent_grid_axes_m=[xmin,xmax,ymin,ymax],known_mask_sha256=maskhash)
     key=hashlib.sha256(json.dumps(pattern,sort_keys=True).encode()).hexdigest()[:16]
     patterns.setdefault(key,pattern);stats[key]+=1
     center=None
     if axis_aligned and n:
      row=int(math.floor(-o.y/res));col=max(0,int(math.floor(-o.x/res)))
      if 0<=row<ht and 0<=col<w and known[row,col]:
       missing=np.flatnonzero(~known[row,col:]);stop=col+int(missing[0]) if len(missing) else w;center=o.x+stop*res
     yfree,xfree=np.where(free);freeend=o.x+(int(xfree.max())+1)*res if len(xfree) else None
     record=dict(bag=bname,db=p.name,message_id=mid,timestamp_ns=ns,time_kst=clock(ns),header_stamp_ns=stamp,pattern_id=key,known_cells=n,unknown_cells=unknown,free_cells=zero,nonzero_known_cells=n-zero,x_min_m=xmin,x_max_m=xmax,y_min_m=ymin,y_max_m=ymax,bbox_holes=int(holes),centerline_known_contiguous_end_m=center,free_x_max_m=freeend)
     writer.writerow(record)
     if key!=previous:
      if interval:runs.append(interval)
      interval=dict(bag=bname,pattern_id=key,first_time_kst=clock(ns),last_time_kst=clock(ns),frames=1)
     else:interval['last_time_kst']=clock(ns);interval['frames']+=1
     previous=key
     isref=axis_aligned and meta['frame']=='Vehicle' and n==13144 and bbox==[0,105,3,126] and abs(res-.3125)<1e-10 and abs(o.x)<1e-10 and abs(o.y+20)<1e-10 and holes==0
     if not isref:exceptions.append(record)
     if freeend is not None:freeends.append(freeend)
     if center is not None:centerends.append(center)
     stampzeros+=stamp==0;frames+=1;read+=1;first=ns if first is None else first;last=ns
    assert read==expected,(bname,read,expected)
   con.close();after=p.stat();sources.append(dict(path=str(p),size_before=before.st_size,size_after=after.st_size,mtime_ns_before=before.st_mtime_ns,mtime_ns_after=after.st_mtime_ns,source_changed=(before.st_size,before.st_mtime_ns)!=(after.st_size,after.st_mtime_ns),ordered_grid_message_stream_sha256=stream.hexdigest()))
  if interval:runs.append(interval)
  summary=dict(bag=bname,frames=frames,first_time_kst=clock(first) if first else None,last_time_kst=clock(last) if last else None,pattern_counts=dict(stats),zero_header_stamps=stampzeros,free_x_max_range_m=[min(freeends),max(freeends)] if freeends else None,centerline_known_contiguous_end_range_m=[min(centerends),max(centerends)] if centerends else None,primary_comparison_bag=bname.startswith('drive_debug_20261006_') and bname.split('_')[3] in primary)
  summaries.append(summary);print(bname,frames,dict(stats),flush=True)
(H/'patterns.json').write_text(json.dumps(patterns,indent=2)+'\n');(H/'summary.json').write_text(json.dumps(summaries,indent=2)+'\n');(H/'pattern_runs.json').write_text(json.dumps(runs,indent=2)+'\n');(H/'sources.json').write_text(json.dumps(sources,indent=2)+'\n')
with (H/'exceptions_from_33m_reference.csv').open('w',newline='') as f:
 writer=csv.DictWriter(f,fieldnames=fields);writer.writeheader();writer.writerows(exceptions)
print('TOTAL',sum(s['frames'] for s in summaries),'PATTERNS',len(patterns),'EXCEPTIONS',len(exceptions),'SOURCE_CHANGED',sum(s['source_changed'] for s in sources),flush=True)
