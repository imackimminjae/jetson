from pathlib import Path
import json,pickle
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
font_manager.fontManager.addfont('/usr/share/fonts/truetype/nanum/NanumGothic.ttf')
plt.rcParams.update({'font.family':'NanumGothic','axes.unicode_minus':False})

root=Path(__file__).resolve().parent
baseline=pickle.load(open(root.parent/'drive_debug_20261001_184521_light/analysis/decoded.pkl','rb'))
old=np.array([z['data'] for z in baseline['/debug/lower_mpc_trace']])

def control_metrics(a,mask):
    v=a[mask]
    idx=np.flatnonzero(mask)
    pair=np.diff(idx)==1
    delta=np.degrees(np.diff(v[:,26]))[pair]
    dt=np.diff(v[:,0])[pair]
    return {'cycles':len(v),'speed_median':float(np.median(v[:,4])),
            'lateral_error_abs_m_median_p95_max':np.quantile(abs(v[:,6]),[.5,.95,1]).tolist(),
            'lateral_error_rms_m':float(np.sqrt(np.mean(v[:,6]**2))),
            'steering_abs_deg_p95_max':np.quantile(abs(np.degrees(v[:,26])),[.95,1]).tolist(),
            'saturation_count':int(np.sum(abs(v[:,26])>.29845)),
            'steering_variation_deg_per_sec':float(np.sum(abs(delta))/sum(dt)),
            'steering_step_abs_deg_p95_max':np.quantile(abs(delta),[.95,1]).tolist()}

all_metrics={'baseline_1845':control_metrics(old,(old[:,10]==1)&(old[:,4]>4))}
for p in sorted(root.glob('sih_fixed*')):
    if not (p/'records.json').exists():continue
    d=json.loads((p/'records.json').read_text());g=json.loads((p/'fixed_path.json').read_text())
    a=np.array(d['trace']);o=np.array(d['pose']);path=np.array(g['xy']);s=np.array(g['s']);kap=np.array(g['kappa'])
    start=d['result']['run_start'];end=a[a[:,10]==1,0].max()
    active=(a[:,10]==1)&(a[:,0]>=start)
    progress=s[np.clip(a[:,5].astype(int),0,len(s)-1)]
    metrics={'result':d['result'],'control_active':control_metrics(a,active),
             'control_speed_over_4':control_metrics(a,active&(a[:,4]>4)),
             'guard_states':sorted(set(x['sih_heading_guard_state'] for x in d['status'])),
             'max_yaw_error_deg':max(abs(x['ekf_minus_sih_yaw_deg']) for x in d['status'] if x['ekf_minus_sih_yaw_deg'] is not None),
             'path_sha256':g['sha256'],'curvature_max_per_m':float(max(abs(kap))),
             'fixed_path_length_m':float(s[-1])}
    for phase,mask in [('straight',(progress<30)|((progress>85)&(progress<90))|(progress>150)),
                       ('left_constant',(progress>52)&(progress<65)),('right_constant',(progress>112)&(progress<125))]:
        valid=active&mask&(a[:,4]>4)
        if np.sum(valid)>2:metrics[phase]=control_metrics(a,valid)
    for name in ['board_after_retry.json','board_after.json']:
        if (p/name).exists():metrics['board_after']=json.loads((p/name).read_text());break
    (p/'metrics.json').write_text(json.dumps(metrics,indent=2,ensure_ascii=False));all_metrics[p.name]=metrics

    fig,axs=plt.subplots(2,2,figsize=(14,9),layout='constrained')
    ax=axs[0,0];ax.plot(path[:,0],path[:,1],'--',c='#db9d0f',lw=2,label='고정 경로');ax.plot(o[:,1],o[:,2],c='#176ec1',label='PX4 EKF 자취');ax.set_aspect('equal');ax.legend();ax.set_xlabel('map X (m)');ax.set_ylabel('map Y (m)');ax.set_title('실제 PX4 SIH + 기존 하위 MPC')
    ax=axs[0,1];ax.plot(a[active,0]-start,np.degrees(a[active,26]),label='고정 경로 조향');ax.axhline(np.degrees(.3),ls=':',c='#777');ax.axhline(-np.degrees(.3),ls=':',c='#777');ax.set_ylabel('조향 (deg)');ax.set_title('고정 경로: 조향 출력');ax.legend()
    ax=axs[1,0];ax.plot(a[active,0]-start,a[active,6],label='횡오차');ax.set_ylabel('m');ax.set_title('고정 경로: 국소 횡오차');ax.legend()
    ax=axs[1,1];mask=(old[:,10]==1)&(old[:,4]>4);v=old[mask];ax.plot(v[:,0]-v[0,0],np.degrees(v[:,26]),c='#b55238',label='18:45 재계획 주행 조향');ax.axhline(np.degrees(.3),ls=':',c='#777');ax.axhline(-np.degrees(.3),ls=':',c='#777');ax.set_ylabel('조향 (deg)');ax.set_title('참고: 기존 주행 (도로·곡률은 서로 다름)');ax.legend()
    for ax in [axs[0,1],axs[1,0],axs[1,1]]:ax.set_xlabel('경과 시간 (s)');ax.grid(alpha=.2)
    fig.suptitle('계획을 고정한 분리 시험 / '+p.name+' / '+d['result']['reason'],fontsize=14)
    fig.supxlabel('목표 속도 6m/s, 하위 가중치·조향 모델 유지. 곡률 최대 '+str(max(abs(kap)))+' /m. 기존 주행과 동일 도로 A/B 시험은 아님.',fontsize=10)
    fig.savefig(p/'comparison.png',dpi=150);plt.close(fig)
(root/'comparison_metrics.json').write_text(json.dumps(all_metrics,indent=2,ensure_ascii=False))
print(json.dumps(all_metrics,indent=2,ensure_ascii=False))
