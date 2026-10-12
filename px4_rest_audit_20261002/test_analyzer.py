"""Synthetic verification of causal alignment and evidence requirements, not vehicle simulation."""
import copy
import numpy as np
from analyze_ulog import REQUIRED,evaluate

def fixture():
 t=np.arange(0,12,.1);us=(1e6+t*1e6).astype(np.int64);s={}
 for name,fields in REQUIRED.items():
  s[name]={f:np.zeros(len(t)) for f in fields};s[name]['timestamp']=us.copy()
 s['vehicle_local_position_groundtruth']['vx']=np.where((t>=3)&(t<6),2.,0.)
 s['vehicle_land_detected']['at_rest']=np.where((t>=3)&(t<4),0.,1.)
 s['estimator_status_flags']['cs_vehicle_at_rest']=np.where((t>=3)&(t<4),0.,1.)
 s['vehicle_land_detected']['landed']=np.ones(len(t))
 s['estimator_gps_status']['checks_passed']=np.where((t>=5)&(t<9),0.,1.)
 s['estimator_gps_status']['check_fail_max_horz_drift']=np.where((t>=5)&(t<9),1.,0.)
 s['vehicle_local_position']['xy_valid']=np.where((t>=5.5)&(t<10),0.,1.)
 s['vehicle_local_position']['v_xy_valid']=s['vehicle_local_position']['xy_valid'].copy()
 return s
s=fixture();summary,tab=evaluate(copy.deepcopy(s));assert summary['conclusion']=='moving_rest_mismatch_observed';assert abs(summary['mismatch_intervals'][0]['start_s']-4)<1e-5;assert abs(summary['mismatch_intervals'][0]['end_s']-6)<1e-5
s2=copy.deepcopy(s);s2['vehicle_land_detected']['at_rest'][:]=0;s2['estimator_status_flags']['cs_vehicle_at_rest'][:]=0
assert evaluate(s2)[0]['conclusion']=='no_sustained_mismatch_in_covered_samples' # landed/disarm alone cannot prove rest
s3=copy.deepcopy(s)
for name in ['vehicle_land_detected','estimator_status_flags']:
 s3[name]={k:v[:1] for k,v in s3[name].items()}
assert not evaluate(s3)[0]['mismatch_intervals'] # stale true flags cannot prove a later mismatch
s4=copy.deepcopy(s);del s4['estimator_gps_status']
assert evaluate(s4)[0]['conclusion']=='insufficient_internal_fields'
s5=copy.deepcopy(s)
for name in ['vehicle_land_detected','estimator_status_flags']:
 s5[name]['timestamp']=s5[name]['timestamp']+5_000_000
summary5,table5=evaluate(s5)
assert all(v['start_s']>=5 for v in summary5['mismatch_intervals'])
assert np.all(np.isnan(table5['land.at_rest'][table5['elapsed_s']<5])) # future samples must not be backfilled
s6=copy.deepcopy(s)
s6['estimator_gps_status']['checks_passed'][:]=1 # firmware latches initial acceptance
summary6,_=evaluate(s6)
stop6=summary6['phases'][-1]
assert abs(stop6['first_observed_checks_passed_latch_s']-6)<1e-5
assert abs(stop6['final_continuous_no_gps_failures_from_s']-9)<1e-5
print('PASS: direct rest evidence, known onset/end, landed-only rejection, stale rejection, missing fields, no future backfill, latched GPS acceptance')
