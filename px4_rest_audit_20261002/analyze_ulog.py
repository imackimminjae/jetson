#!/usr/bin/env python3
"""Offline ULog audit. Never connects to ROS, MAVLink or hardware.
Time alignment is causal (last published sample); no interpolation of booleans.
Evidence requires fresh truth and BOTH directly recorded internal rest flags.
"""
import argparse,csv,json
from pathlib import Path
import numpy as np

REQUIRED={
 'vehicle_local_position_groundtruth':['timestamp','x','y','vx','vy'],
 'vehicle_land_detected':['timestamp','landed','ground_contact','at_rest'],
 'estimator_status_flags':['timestamp','cs_vehicle_at_rest','cs_in_air','cs_gnss_pos','cs_gnss_vel'],
 'estimator_gps_status':['timestamp','checks_passed','check_fail_max_horz_drift','check_fail_max_horz_spd_err'],
 'vehicle_local_position':['timestamp','xy_valid','v_xy_valid','x','y','vx','vy','xy_reset_counter','vxy_reset_counter'],
 'actuator_armed':['timestamp','armed','prearmed'],
 'actuator_outputs_sim':['timestamp','noutputs','output[0]','output[1]'],
}
PREFIX={n:p for n,p in zip(REQUIRED,['truth','land','ekf','gps','local','armed','act'])}
MAX_AGE={'truth':.15,'land':1.5,'ekf':1.5,'gps':1.,'local':.15,'armed':1.5}

def spans(mask,t):
 """Intervals end at the first observed false sample, not last true sample."""
 mask=np.asarray(mask,dtype=bool);changes=np.diff(np.r_[False,mask,False].astype(int));starts=np.where(changes==1)[0];ends=np.where(changes==-1)[0]
 return [(float(t[a]),float(t[b]) if b<len(t) else float(t[-1])) for a,b in zip(starts,ends)]

def evaluate(streams):
 missing={n:[f for f in fields if n not in streams or f not in streams[n]] for n,fields in REQUIRED.items()}
 missing={n:fs for n,fs in missing.items() if fs}
 if missing:return {'conclusion':'insufficient_internal_fields','missing':missing},None
 for name,stream in streams.items():
  order=np.argsort(stream['timestamp'],kind='stable');streams[name]={k:np.asarray(v)[order] for k,v in stream.items()}
 truth=streams['vehicle_local_position_groundtruth'];lo=int(truth['timestamp'][0]);hi=int(truth['timestamp'][-1])
 union=np.unique(np.concatenate([s['timestamp'] for s in streams.values()]));union=union[(union>=lo)&(union<=hi)];t=(union-lo)*1e-6
 table={'time_boot_us':union,'elapsed_s':t};events=[]
 for name,prefix in PREFIX.items():
  s=streams[name];ts=s['timestamp'];idx=np.searchsorted(ts,union,side='right')-1;has=idx>=0;safe=np.maximum(idx,0);age=np.where(has,(union-ts[safe])*1e-6,np.nan)
  table[prefix+'_age_s']=age
  fields=list(REQUIRED[name])
  if name=='estimator_gps_status':fields+= [k for k in s if k.startswith('check_fail_') or k.endswith('_m_s')]
  if name=='actuator_outputs_sim':fields += [k for k in s if k.startswith('output[')]
  if name=='vehicle_local_position':fields += [k for k in s if k.endswith('_reset_counter') or k.startswith('delta_')]
  if 'timestamp_sample' in s:fields.append('timestamp_sample')
  for f in dict.fromkeys(fields):
   if f not in s:continue
   v=s[f].astype(float);aligned=np.where(has,v[safe],np.nan);table[prefix+'.'+f]=aligned
   if f.startswith(('cs_','check_fail_')) or f in ['at_rest','landed','ground_contact','xy_valid','v_xy_valid','armed','prearmed','checks_passed'] or f.endswith('_reset_counter'):
    changes=np.r_[0,np.flatnonzero(v[1:]!=v[:-1])+1]
    for j in changes:
     if lo<=ts[j]<=hi:events.append({'elapsed_s':float((int(ts[j])-lo)*1e-6),'time_boot_us':int(ts[j]),'field':prefix+'.'+f,'value':float(v[j]),'initial_sample':bool(j==0)})
 speed=np.hypot(table['truth.vx'],table['truth.vy']);table['truth.speed']=speed;table['local.speed']=np.hypot(table['local.vx'],table['local.vy'])
 fresh={p:np.isfinite(table[p+'_age_s'])&(table[p+'_age_s']<=age) for p,age in MAX_AGE.items()}
 moving=(speed>.2)&fresh['truth'];both=(table['land.at_rest']==1)&(table['ekf.cs_vehicle_at_rest']==1)
 valid=fresh['truth']&fresh['land']&fresh['ekf'];mismatch=moving&both&valid;table['moving_and_both_rest']=mismatch.astype(int)
 intervals=[{'start_s':a,'end_s':b,'duration_s':b-a} for a,b in spans(mismatch,t) if b-a>=.5]
 land_only=spans(moving&(table['land.at_rest']==1)&fresh['land'],t);ekf_only=spans(moving&(table['ekf.cs_vehicle_at_rest']==1)&fresh['ekf'],t)
 phases=[];state=None;start=0
 for i,v in enumerate(speed):
  new=state
  if not fresh['truth'][i]:new='unknown'
  elif v>.2:new='moving'
  elif v<.05:new='stopped'
  if new!=state:
   if state is not None and t[i]-t[start]>=.5:phases.append({'state':state,'start_s':float(t[start]),'end_s':float(t[i])})
   state=new;start=i
 if state is not None and t[-1]-t[start]>=.5:phases.append({'state':state,'start_s':float(t[start]),'end_s':float(t[-1])})
 failures=[k for k in table if k.startswith('gps.check_fail_')]
 for phase in phases:
  if phase['state']!='stopped':continue
  mask=(t>=phase['start_s'])&(t<phase['end_s']);tail=mask&(t>=phase['end_s']-2)&fresh['gps']
  phase['gps_failures_seen']=[k for k in failures if np.any((table[k]==1)&mask&fresh['gps'])]
  phase['gps_failures_in_last_2s']=[k for k in failures if np.any((table[k]==1)&tail)]
  ok=mask&fresh['gps']&(table['gps.checks_passed']==1);phase['first_observed_checks_passed_latch_s']=float(t[np.flatnonzero(ok)[0]]) if np.any(ok) else None
  no_failure=np.logical_and.reduce([table[k]==0 for k in failures])&fresh['gps']
  ok=mask&no_failure;phase['first_observed_no_gps_failure_flags_s']=float(t[np.flatnonzero(ok)[0]]) if np.any(ok) else None
  clear_spans=spans(ok,t);phase['final_continuous_no_gps_failures_from_s']=next((a for a,b in reversed(clear_spans) if b>=phase['end_s']-.02),None)
  ok=mask&fresh['local']&(table['local.xy_valid']==1)&(table['local.v_xy_valid']==1);phase['first_observed_horizontal_valid_s']=float(t[np.flatnonzero(ok)[0]]) if np.any(ok) else None
 summary={'conclusion':'moving_rest_mismatch_observed' if intervals else 'no_sustained_mismatch_in_covered_samples','duration_s':float(t[-1]),'time_origin_boot_us':lo,'moving_threshold_m_s':.2,'stopped_threshold_m_s':.05,'minimum_mismatch_s':.5,'fresh_moving_rest_coverage':float(np.mean(valid[moving])) if np.any(moving) else None,'mismatch_intervals':intervals,'land_at_rest_while_moving_intervals_s':land_only,'ekf_at_rest_while_moving_intervals_s':ekf_only,'phases':phases,'events':sorted(events,key=lambda e:e['time_boot_us']),'maximum_actuator_message_age_s':float(np.nanmax(table['act_age_s'])) if np.any(np.isfinite(table['act_age_s'])) else None,'checks_passed_semantics':'This PX4 commit latches checks_passed after initial health acceptance. Use individual check_fail_* flags for current GNSS quality.', 'scope':'Observed topic publication times, not exact internal execution times; inspect timestamp_sample and logger dropouts. Landed/disarm/constant-position are not used as evidence of rest.'}
 return summary,table

def plot(table,path):
 import matplotlib
 matplotlib.use('Agg')
 import matplotlib.pyplot as plt
 t=table['elapsed_s'];fig,axs=plt.subplots(6,1,figsize=(14,14),sharex=True)
 for k in ['truth.speed','local.speed']:axs[0].plot(t,table[k],label=k)
 for k in ['land.at_rest','ekf.cs_vehicle_at_rest','land.landed','ekf.cs_in_air']:axs[1].step(t,table[k],where='post',label=k)
 fail=[k for k in table if k.startswith('gps.check_fail_')];mat=np.stack([table[k] for k in fail]);axs[2].pcolormesh(t,np.arange(len(fail)),mat,shading='nearest',vmin=0,vmax=1,cmap='Reds');axs[2].set_yticks(range(len(fail)),[x.replace('gps.check_fail_','') for x in fail],fontsize=7)
 for k in ['local.xy_valid','local.v_xy_valid','ekf.cs_gnss_pos','ekf.cs_gnss_vel']:axs[3].step(t,table[k],where='post',label=k)
 for k in ['act.output[0]','act.output[1]','armed.armed','armed.prearmed']:axs[4].step(t,table[k],where='post',label=k)
 age_axis=axs[4].twinx();age_axis.plot(t,table['act_age_s'],color='gray',alpha=.4,label='actuator age');age_axis.set_ylabel('Input age (s)',color='gray')
 for k in ['local.xy_reset_counter','local.vxy_reset_counter']:axs[5].step(t,table[k],where='post',label=k)
 for i,ax in enumerate(axs):
  ax.grid(alpha=.2)
  if i!=2:ax.legend(loc='upper left',fontsize=8,ncol=4)
 axs[0].set_ylabel('m/s');axs[-1].set_xlabel('Seconds from first SIH groundtruth sample');fig.tight_layout();fig.savefig(path,dpi=140);plt.close(fig)

def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('ulog',type=Path);p.add_argument('--output',type=Path,required=True);p.add_argument('--estimator-instance',type=int,default=0);a=p.parse_args()
 from pyulog import ULog
 u=ULog(str(a.ulog));streams={}
 for name in REQUIRED:
  instance=a.estimator_instance if name.startswith('estimator_') else 0
  matches=[d for d in u.data_list if d.name==name and d.multi_id==instance and len(d.data.get('timestamp',[]))]
  if matches:streams[name]=matches[0].data
 summary,table=evaluate(streams);summary['ulog']=str(a.ulog);summary['firmware_info']={k:str(v) for k,v in u.msg_info_dict.items() if 'ver_' in k};summary['estimator_instance']=a.estimator_instance;summary['dropouts']=[{'timestamp_us':int(d.timestamp),'duration_ms':int(d.duration)} for d in u.dropouts]
 selectors=[d for d in u.data_list if d.name=='estimator_selector_status'];summary['selected_estimator_instances']=sorted({int(v) for d in selectors for v in d.data.get('primary_instance',[])})
 if summary['selected_estimator_instances'] and summary['selected_estimator_instances']!=[a.estimator_instance]:summary['primary_estimator_warning']='Primary estimator differs or switches: analyze each instance; local-position validity cannot be causally attributed to this instance alone.'
 a.output.mkdir(parents=True,exist_ok=True);(a.output/'summary.json').write_text(json.dumps(summary,indent=2,allow_nan=False))
 if table is not None:
  with (a.output/'aligned.csv').open('w') as f:
   w=csv.writer(f);w.writerow(table);w.writerows(zip(*table.values()))
  plot(table,a.output/'timeline.png')
 print(json.dumps({k:v for k,v in summary.items() if k not in ['events','firmware_info']},indent=2))
if __name__=='__main__':main()
