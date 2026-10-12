from pathlib import Path
from collections import Counter
from datetime import datetime
from zoneinfo import ZoneInfo
import os,json,gzip
os.sched_setaffinity(0,{min(os.sched_getaffinity(0))})
import numpy as np
H=Path(__file__).resolve().parent
clock=lambda t:datetime.fromtimestamp(float(t),ZoneInfo('Asia/Seoul')).strftime('%H:%M:%S.%f')[:-3]
allrows=[]
for sc in [1,2,3]:
 p=H/f'scenario{sc}';s=json.loads((p/'summary.json').read_text());es=json.loads(gzip.decompress((p/'events.json.gz').read_bytes()));d=json.loads(gzip.decompress((p/'telemetry.json.gz').read_bytes()));facts=[]
 for z in s['attempts']:
  ee=[e for e in es if z['start']<=e['t_start']<z['end']];trans=[];prev=None;bad_runs=[];begin=None
  for e in ee:
   key=(e['valid_plan'],e['reason'])
   if key!=prev:trans.append(dict(t=e['t_start'],time=clock(e['t_start']),xy=[e['x'],e['y']],valid=e['valid_plan'],reason=e['reason'],horizon=e['horizon'],end=e['published_world'][-1:]));prev=key
   if not e['valid_plan'] and begin is None:begin=e['t_start']
   if e['valid_plan'] and begin is not None:bad_runs.append(dict(start=clock(begin),end=clock(e['t_start']),duration=e['t_start']-begin,recovered=True));begin=None
  if begin is not None:bad_runs.append(dict(start=clock(begin),end=z['end_clock'],duration=z['end']-begin,recovered=False))
  bt=np.array([a for _,a in d['data']['/debug/upper_interval_centering_state']]);bchanges=[]
  for b in bt[(bt[:,0]>=z['start'])&(bt[:,0]<z['end'])]:
   if abs(b[2]-b[1])>2:bchanges.append(dict(time=clock(b[0]),before=b[1],after=b[2]))
  f=dict(attempt=z['attempt'],phase_transitions=trans,failure_runs=bad_runs,large_B_changes=bchanges,guard_events=[dict(time=clock(e['t_start']),xy=[e['x'],e['y']],tail=e['side_boundary_tail_start']) for e in ee if e.get('side_boundary_tail_start',-1)>=0]);facts.append(f)
 no=[z for z in s['attempts'] if z['outcome']=='completed'];stats=dict(scenario=sc,outcomes=dict(Counter(z['outcome'] for z in s['attempts'])),total_upper=sum(z['upper_count'] for z in s['attempts']),failed_upper=sum(z['upper_failed'] for z in s['attempts']),guard_hits=sum(z['guard_hits'] for z in s['attempts']),max_odom_gap=max(z['gaps']['/motive/vehicle/odom_map']['seconds'] for z in s['attempts']),max_grid_gap=max(z['gaps']['/bev/occupancy_grid']['seconds'] for z in s['attempts']),sat_range=[min(z['sat_percent'] for z in s['attempts']),max(z['sat_percent'] for z in s['attempts'])],weighted_sat_percent=sum(z['sat_percent']*z['active_samples'] for z in s['attempts'])/sum(z['active_samples'] for z in s['attempts']),lateral_p95_range=[min(z['lateral_abs']['p95'] for z in s['attempts']),max(z['lateral_abs']['p95'] for z in s['attempts'])],goal_stop_range=[min(float(z['stop_goal_distance_m']) for z in no),max(float(z['stop_goal_distance_m']) for z in no)] if no else None,neutral_to_stop_displacement_range=[min(z['terminal_neutral']['displacement_to_stop_m'] for z in no),max(z['terminal_neutral']['displacement_to_stop_m'] for z in no)] if no else None,neutral_to_next_phase_seconds_range=[min(z['terminal_neutral']['seconds_to_phase_end'] for z in no),max(z['terminal_neutral']['seconds_to_phase_end'] for z in no)] if no else None)
 (p/'failure_timeline.json').write_text(json.dumps(facts,indent=2));allrows.append(stats);print(stats)
 for z in s['attempts']:print('SC',sc,'attempt',z['attempt'],'B',z['B_range'],'guard',z['guard_hits'],'k1',z['largest_k1_shifts'][:1])
(H/'cross_scenario_summary.json').write_text(json.dumps(allrows,indent=2))
