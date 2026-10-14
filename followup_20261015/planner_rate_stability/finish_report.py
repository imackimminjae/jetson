from pathlib import Path
import csv,json,pickle,hashlib
import numpy as np
import matplotlib
matplotlib.use('Agg')
from matplotlib import pyplot as plt,font_manager
from matplotlib.backends.backend_pdf import PdfPages
from PIL import Image
OUT=Path(__file__).resolve().parent
font_manager.fontManager.addfont('/usr/share/fonts/truetype/nanum/NanumGothic.ttf')
plt.rcParams.update({'font.family':'NanumGothic','axes.unicode_minus':False})
summaries=json.loads((OUT/'summary.json').read_text());reports=json.loads((OUT/'metrics.json').read_text())
fig,axs=plt.subplots(1,4,figsize=(15,4.5))
colors=['#7b8794','#4477aa','#009988','#cc6677'];xs=np.arange(4)
specs=[('variation_deg_per_100m','누적 조향 보정량\n[도 / 100 m]',True),('rms_spread_m','회차별 궤적 편차 RMS\n[m]',False),('rapid_residual_rms_deg','빠른 조향 변화 RMS\n[도 환산]',True),('yaw_accel_rms_degps2','차량 yaw 가속도 RMS\n[도 / 초²]',True)]
for ax,(key,title,has_runs) in zip(axs,specs):
    med=[s[key]['median'] if has_runs else s['repeatability'][key] for s in summaries]
    ax.bar(xs,med,color=colors,width=.6,alpha=.8)
    if has_runs:
        for j,r in enumerate(reports):
            vals=[a['steering'][key] if key in a['steering'] else a[key] for a in r['attempts']]
            ax.scatter(j+np.linspace(-.13,.13,5),vals,s=22,color='#333333',zorder=3)
    ax.set_xticks(xs,[f"{r['rate_hz']:g} Hz" for r in reports]);ax.set_title(title);ax.grid(axis='y',alpha=.2);ax.set_axisbelow(True)
    for x,y in zip(xs,med):ax.text(x,y*.45,f'{y:.2f}' if y<10 else f'{y:.1f}',ha='center',color='white',fontweight='bold')
fig.suptitle('시나리오1 · 각 5회 · 점: 개별 회차 / 궤적 편차: 5회 종합 / 나머지 막대: 중앙값',fontsize=14)
fig.tight_layout();fig.savefig(OUT/'overview.png',dpi=165)
with PdfPages(OUT/'stability_comparison.pdf') as pdf:
    pdf.savefig(fig)
    for name in ['trajectories.png','spatial_signals.png']:
        f,ax=plt.subplots(figsize=(16,10));ax.imshow(plt.imread(OUT/name));ax.axis('off');f.tight_layout();pdf.savefig(f);plt.close(f)

# Exclude the single 1 Hz attempt with a long BEV interruption as a sensitivity check.
r=reports[0];keep=[a for a in r['attempts'] if a['grid_gap_max_s']<=.5]
d=pickle.load((OUT/f"decoded_{r['batch']}.pkl").open('rb'));od=d['data']['/motive/vehicle/odom_map'];route=np.array(d['route']);vec=np.diff(route,axis=0);length=np.linalg.norm(vec,axis=1);cum=np.r_[0,np.cumsum(length)];sg=np.arange(5,180,.5);tracks=[]
for a in keep:
    tr=np.array([v for v in od if a['start']<=v[0]<a['end']]);pos=tr[:,1:3];delta=pos[:,None,:]-route[:-1][None,:,:];u=np.clip(np.sum(delta*vec[None,:,:],axis=2)/(length**2)[None,:],0,1);foot=route[:-1][None,:,:]+u[:,:,None]*vec[None,:,:];idx=np.argmin(np.sum((pos[:,None,:]-foot)**2,axis=2),axis=1);s=cum[idx]+u[np.arange(len(idx)),idx]*length[idx];ss,j=np.unique(np.maximum.accumulate(s),return_index=True);tracks.append(np.column_stack([np.interp(sg,ss,tr[j,k]) for k in [1,2]]))
tr=np.array(tracks);spread=np.sqrt(np.var(tr[:,:,0],axis=0)+np.var(tr[:,:,1],axis=0))
sensitivity={'rate_hz':1,'included_attempts':[a['attempt'] for a in keep],'reason':'Exclude attempt5 with max BEV reception gap 1.6585s','steering_variation_deg_per_100m_median':float(np.median([a['steering']['variation_deg_per_100m'] for a in keep])),'trajectory_spread_rms_m':float(np.sqrt(np.mean(spread**2)))}
(OUT/'sensitivity.json').write_text(json.dumps(sensitivity,indent=2)+'\n')
print('SENSITIVITY',sensitivity)
for p,h in json.loads((OUT/'sources_sha256.json').read_text()).items():assert hashlib.sha256(Path(p).read_bytes()).hexdigest()==h
for p in OUT.glob('*.png'):
    with Image.open(p) as im:im.verify()
v=json.loads((OUT/'validation.json').read_text());v.update(original_hashes_rechecked=True,png_verified=True,all_attempts_completed=20,steering_metric='Actual sent MAVLink command x configured 0.35 rad/normalized unit; not measured actuator angle',spatial_comparison='Same waypoint-route progress 5..179.5m at0.5m intervals',sensitivity=sensitivity)
(OUT/'validation.json').write_text(json.dumps(v,ensure_ascii=False,indent=2)+'\n')
print('Report figures and validation complete')
