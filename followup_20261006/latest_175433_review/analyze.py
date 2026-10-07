from pathlib import Path
import pickle,json,hashlib,csv,os
from collections import Counter
from datetime import datetime
from zoneinfo import ZoneInfo
import numpy as np,yaml
os.environ['MPLCONFIGDIR']='/tmp/codex_mpl_grid'
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
font_manager.fontManager.addfont('/usr/share/fonts/truetype/nanum/NanumGothic.ttf');plt.rcParams.update({'font.family':'NanumGothic','axes.unicode_minus':False})
H=Path(__file__).resolve().parent;R=H.parents[1];tags=['174526','175433'];all_s={};all_data={};manifest={}
def clock(t):return datetime.fromtimestamp(float(t),ZoneInfo('Asia/Seoul')).strftime('%H:%M:%S.%f')[:-3]
def quant(a):return dict(zip(['min','median','p95','max'],map(float,np.quantile(a,[0,.5,.95,1])))) if len(a) else {}
def proj(points,line):
 v=np.diff(line,axis=0);l=np.linalg.norm(v,axis=1);u=np.clip(np.sum((points[:,None,:]-line[None,:-1,:])*v,axis=2)/(l*l),0,1);q=line[None,:-1,:]+u[:,:,None]*v;dist=np.linalg.norm(points[:,None,:]-q,axis=2);j=np.argmin(dist,axis=1);k=np.arange(len(points));s=np.r_[0,np.cumsum(l)];return s[j]+u[k,j]*l[j],dist[k,j]
def gaps(t,threshold):return [{'from':clock(t[i]),'to':clock(t[i+1]),'sec':float(t[i+1]-t[i])} for i in np.flatnonzero(np.diff(t)>threshold)]
for tag in tags:
 p=R/f'drive_debug_20261006_{tag}_light';o=H/tag;o.mkdir(exist_ok=True);d=pickle.load((p/'analysis/decoded.pkl').open('rb'));g=d['/bev/occupancy_grid'];es=[v['data'] for v in d['/debug/upper_branch_event']];tr=np.array([v['data'] for v in d['/debug/lower_mpc_trace']]);od=d['/motive/vehicle/odom_map'];ot=np.array([v['t'] for v in od]);xy=np.array([v['xy'] for v in od]);gp=d['/navigation/global_path'][0]['xy'];et=np.array([e['t_start'] for e in es]);gt=np.array([v['t'] for v in g]);prog,dist=proj(xy,gp);lp,ld=proj(tr[:,1:3],gp);active=(tr[:,12]==0)&(tr[:,4]>1);cfg=yaml.safe_load((p/'config_snapshot.yaml').read_text());gridrows=[]
 for m in g:
  a=m['grid'];yy,xx=np.where(a==0);ky,kx=np.where(a>=0);r=m['res'];ox,oy=m['origin'];gridrows.append({'time':clock(m['t']),'t':m['t'],'known':int(len(kx)),'unknown':int((a<0).sum()),'free':len(xx),'known_far_received':float(ox+(kx.max()+1)*r) if len(kx) else None,'free_far_effective0':float((xx.max()+1)*r) if len(xx) else None,'free_first9m':int((a[:,:int(9/r)]==0).sum()),'free_at_40m_edge':int((a[:,-1]==0).sum())})
 with (o/'all_grid_frames.csv').open('w') as f:w=csv.DictWriter(f,fieldnames=gridrows[0]);w.writeheader();w.writerows(gridrows)
 plan=[];prev=None
 for e in es:
  row={'time':clock(e['t_start']),'t':e['t_start'],'xy':[e['x'],e['y']],'valid':e['valid_plan'],'reason':e['reason'],'N':e['horizon'],'wp':[e['wp0'],e['wp1']],'counts':e['candidate_counts'],'selected':e['selected_corridors'],'k1_shift':None,'k1_outside':None,'k1_widths':None,'end':None,'ref_end':e['preview_world'][-1] if e['preview_world'] else None}
  if e['valid_plan']:
   pl=np.array(e['planned_world']);ref=np.array(e['preview_world']);row['end']=pl[-1].tolist()
   if prev is not None and e['t_start']-prev[0]<.8:
    v=np.diff(prev[1],axis=0);l=np.linalg.norm(v,axis=1);u=np.clip(np.sum((pl[1]-prev[1][:-1])*v,axis=1)/(l*l),0,1);q=prev[1][:-1]+u[:,None]*v;j=np.argmin(np.linalg.norm(pl[1]-q,axis=1));row['k1_shift']=float(np.cross(v[j],pl[1]-q[j])/l[j])
   sel=e['selected_corridors'][1];iv=next((r for r in e['intervals_world'] if r[0]==1 and r[1]==sel),None)
   if iv:
    a=np.array(iv[2:4]);b=np.array(iv[4:6]);le=np.linalg.norm(b-a);s=float((ref[1]-a)@(b-a)/le);row['k1_outside']=max(-s,s-le,0);row['k1_widths']=iv[6:8]
   prev=(e['t_start'],pl)
  plan.append(row)
 segs={}
 for label,lo,hi in [('공통 초반 0~45m',0,45),('공통 첫 분기 45~85m',45,85),('두 번째 분기 접근 85~125m',85,125)]:
  mask=active&(lp>=lo)&(lp<hi);x=tr[mask];ii=np.flatnonzero(mask);changes=sum(np.sign(tr[b,26])!=np.sign(tr[a,26]) and abs(tr[a,26])>np.radians(2) and abs(tr[b,26])>np.radians(2) for a,b in zip(ii[:-1],ii[1:]) if tr[b,0]-tr[a,0]<.2)
  segs[label]={'n':len(x),'speed':quant(x[:,4]),'sat_fraction':float(np.mean(abs(x[:,26])>=.29845)) if len(x) else None,'local_error_abs':quant(abs(x[:,6])),'steering_deg':quant(np.degrees(x[:,26])),'heading_abs_deg':quant(abs(np.degrees(x[:,7])))}
 s={'bag':str(p),'bag_grid_window':[clock(gt[0]),clock(gt[-1])],'pose_window':[clock(ot[0]),clock(ot[-1])],'route':gp.tolist(),'goals_true':sum(bool(v['data']) for v in d['/planner/goal_reached']),'last_xy':xy[-1].tolist(),'last_goal_distance_m':float(np.linalg.norm(xy[-1]-gp[-1])),'upper_n':len(es),'upper_valid':sum(e['valid_plan'] for e in es),'upper_failures':[{'time':clock(e['t_start']),'xy':[e['x'],e['y']],'status':e['status']} for e in es if not e['valid_plan']],'horizons':dict(Counter(e['horizon'] for e in es)),'upper_gaps':gaps(et,.8),'path_gaps':gaps(np.array([v['t'] for v in d['/planner/lower_reference_path/with_arclength']]),.8),'lower_gaps':gaps(tr[:,0],.3),'odom_max_gap_sec':float(max(np.diff(ot))),'grid_n':len(g),'grid_max_gap_sec':float(max(np.diff(gt))),'geometry_patterns':dict(Counter(str((m['width'],m['height'],m['res'],m['origin'],m['q'])) for m in g)),'known_counts':dict(Counter(r['known'] for r in gridrows)),'unknown_counts':dict(Counter(r['unknown'] for r in gridrows)),'grid_edge_40m_free_frames':sum(r['free_at_40m_edge']>0 for r in gridrows),'near9m_empty':sum(r['free_first9m']==0 for r in gridrows),'free_far_effective0':quant([r['free_far_effective0'] for r in gridrows if r['free_far_effective0'] is not None]),'all_lower_modes':dict(Counter(map(int,tr[:,12]))),'moving_valid_n':int(active.sum()),'moving_valid_sat_fraction':float(np.mean(abs(tr[active,26])>=.29845)),'moving_valid_lateral_error_abs':quant(abs(tr[active,6])),'segments':segs,'largest_shifts':sorted([r for r in plan if r['k1_shift'] is not None],key=lambda r:abs(r['k1_shift']),reverse=True)[:6],'snapshot_config':{'upper':cfg['upper_planner_node']['ros__parameters'],'lower':cfg['lower_tracking_mpc_node']['ros__parameters']}}
 # contiguous invalid runs; do not bridge recording gaps
 inv=[];start=None
 for i,row in enumerate(tr):
  bad=row[12]!=0
  if start is not None and (not bad or (i>0 and row[0]-tr[i-1,0]>.3)):
   inv.append({'start':clock(tr[start,0]),'end':clock(tr[i-1,0]),'n':i-start,'mode':int(tr[start,12])});start=None
  if bad and start is None:start=i
 if start is not None:inv.append({'start':clock(tr[start,0]),'end':clock(tr[-1,0]),'n':len(tr)-start,'mode':int(tr[start,12])})
 s['invalid_runs']=inv
 (o/'summary.json').write_text(json.dumps(s,ensure_ascii=False,indent=2));(o/'plan_events.json').write_text(json.dumps(plan,ensure_ascii=False,indent=2));(o/'rosout.txt').write_text('\n'.join(clock(v['t'])+' '+v['name']+' '+v['msg'] for v in d['/rosout']));all_s[tag]=s;all_data[tag]={'d':d,'tr':tr,'xy':xy,'ot':ot,'gp':gp,'es':es,'et':et,'prog':prog,'lp':lp,'plan':plan,'grid':gridrows}
 for f in [p/'bag/bag_0.db3',p/'bag/metadata.yaml',p/'config_snapshot.yaml']:manifest[str(f)]=hashlib.sha256(f.read_bytes()).hexdigest()
 print(tag,json.dumps({k:v for k,v in s.items() if k not in ['snapshot_config','largest_shifts','route']},ensure_ascii=False,indent=2))
(H/'sources_sha256.json').write_text(json.dumps(manifest,indent=2));(H/'summary.json').write_text(json.dumps(all_s,ensure_ascii=False,indent=2))
fig,axs=plt.subplots(2,2,figsize=(14,10),layout='constrained');cols=['#c36b21','#256fbb']
for tag,col in zip(tags,cols):
 z=all_data[tag];label=tag[:2]+':'+tag[2:4]+':'+tag[4:];xy=z['xy'];t=z['tr'];lp=z['lp'];axs[0,0].plot(*xy.T,c=col,label=label);valid=(t[:,12]==0)&(t[:,4]>1);y=np.where(valid,np.degrees(t[:,26]),np.nan);dis=np.r_[False,np.diff(t[:,0])>.3];y[dis]=np.nan;axs[0,1].plot(lp,y,c=col,label=label,lw=1)
 p=[r for r in z['plan'] if r['k1_shift'] is not None];pp,_=proj(np.array([r['xy'] for r in p]),z['gp']);ps=np.array([r['k1_shift'] for r in p]); ps[np.r_[False,np.diff([r['t'] for r in p])>.8]]=np.nan; axs[1,0].plot(pp,ps,'.-',c=col,label=label,lw=.9)
 err=np.where(valid,abs(t[:,6]),np.nan); err[dis]=np.nan; axs[1,1].plot(lp,err,c=col,label=label,lw=1)
axs[0,0].plot(*all_data[tags[0]]['gp'].T,'k--o',ms=4,label='지정 경로');axs[0,0].set_aspect('equal');axs[0,0].set_title('두 주행 모두 마지막 북동쪽 분기 이탈');axs[0,0].set_xlabel('지도 X (m)');axs[0,0].set_ylabel('지도 Y (m)');axs[0,0].legend(fontsize=9)
for ax,title,ylabel in [(axs[0,1],'동일 경로 진행량에서 조향 비교','ROS 출력 조향 (도)'),(axs[1,0],'새 근거리 k1의 직전 계획 대비 이동','횡이동 (m)'),(axs[1,1],'정상 계산 중 지역 경로 추종 오차','횡오차 절댓값 (m)')]:
 ax.set_title(title);ax.set_ylabel(ylabel);ax.set_xlabel('지정 경로에 투영한 진행 거리 (m)');ax.set_xlim(0,135);ax.grid(alpha=.2);ax.legend(fontsize=8);ax.axvline(125,c='gray',ls=':')
fig.suptitle('최신 두 scenario3 주행 · 도착 없음 · 공백/중립은 조향 비교에서 제외\n125m 이후 오분기 궤적은 같은 도로 구간으로 비교하지 않음',fontsize=14);fig.savefig(H/'comparison.png',dpi=145);plt.close(fig)
fig,axs=plt.subplots(1,2,figsize=(13,6),layout='constrained')
for ax,tag in zip(axs,tags):
 z=all_data[tag];ax.plot(*z['gp'].T,'k--o',ms=3,label='지정 경로');ax.plot(*z['xy'].T,c='gray',label='차량 궤적');ss=[e for e in z['es'] if e['valid_plan'] and 225<e['x']<257]
 for i,e in enumerate(ss):
  pl=np.array(e['planned_world']);ax.plot(*pl.T,'.-',ms=2,lw=1,color=plt.cm.viridis(i/max(1,len(ss)-1)),label=clock(e['t_start']) if i in [0,len(ss)-1] else None)
 ax.set_xlim(225,291);ax.set_ylim(118,177);ax.set_aspect('equal');ax.set_title(tag+' · 두 번째 분기 연속 계획');ax.set_xlabel('지도 X (m)');ax.set_ylabel('지도 Y (m)');ax.legend(fontsize=8);ax.grid(alpha=.2)
fig.suptitle('상위 계획 자체가 남쪽 갈래로 이어짐 · 목표는 (270,158) → (278,172)',fontsize=14);fig.savefig(H/'branch_plans.png',dpi=145);plt.close(fig)
