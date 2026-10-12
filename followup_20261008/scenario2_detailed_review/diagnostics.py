from pathlib import Path
import json,gzip,os
os.sched_setaffinity(0,{min(os.sched_getaffinity(0))});os.environ['MPLCONFIGDIR']='/tmp/scenario2_detail_diagnostic'
import numpy as np
H=Path(__file__).resolve().parent;P=H.parent/'scenario2_causes';s=json.loads((H/'summary.json').read_text());checks={}
for i in range(1,8):
 c=json.loads((P/f'input_{i}.json').read_text())['cycles'];r=json.loads(gzip.decompress((P/f'result_{i}.json.gz').read_bytes()));err=[]
 for a,b in zip(c,r['fixed']):
  if a['rec_valid'] and b['valid'] and np.array(a['rec_planned']).shape==np.array(b['planned']).shape:err.append(float(np.linalg.norm(np.array(a['rec_planned'])-b['planned'],axis=1).max()))
 checks[i]=dict(cycles=len(c),valid_match=sum(a['rec_valid']==b['valid'] for a,b in zip(c,r['fixed'])),valid_comparable=len(err),plan_within_1mm=sum(x<.001 for x in err),plan_max_error_m=max(err),mode_failures={m:sum(not z['valid'] for z,a in zip(rows,c) if a['running']) for m,rows in r.items()})
(H/'offline_validation.json').write_text(json.dumps(dict(source=str(P),checks=checks,branch=json.loads((P/'branch_comparison.json').read_text()),feasibility=json.loads((P/'branch_feasibility.json').read_text()),rotation_gaps=json.loads((P/'rotation_gaps.json').read_text())),indent=2))
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
font_manager.fontManager.addfont('/usr/share/fonts/truetype/nanum/NanumGothic.ttf');plt.rcParams.update({'font.family':'NanumGothic','axes.unicode_minus':False})
fig,axs=plt.subplots(1,2,figsize=(14,6),layout='constrained');route=np.array(s['route']);tr=json.loads(gzip.decompress((H/'tracks.json.gz').read_bytes()));axs[0].plot(*route.T,'k--o',ms=3,label='지정 경로')
for i,col in [(1,'#148676'),(7,'#c44632')]:
 a=np.array(tr[i-1]['xy']);tt=np.array(tr[i-1]['t']);mask=(tt<tr[i-1]['end'])&(a[:,0]>-172)&(a[:,0]<-111);inds=np.flatnonzero(mask)
 for j,idx in enumerate(np.split(inds,np.flatnonzero(np.diff(tt[inds])>.5)+1)):axs[0].plot(*a[idx].T,color=col,label=f'{i}회 실측' if j==0 else None)
c7=json.loads((P/'input_7.json').read_text())['cycles'];r7=json.loads(gzip.decompress((P/'result_7.json.gz').read_bytes()))
for j,col in [(49,'#de9922'),(50,'#c44632')]:
 a=np.array(c7[j]['rec_published']);axs[0].plot(*a.T,'o-',color=col,ms=3,label=f'7회 계획 {j-49+1}')
axs[0].set_xlim(-170,-111);axs[0].set_ylim(27,77);axs[0].set_aspect('equal');axs[0].grid(alpha=.2);axs[0].legend(fontsize=8);axs[0].set_title('7회: 중간 후보 소실 이후 남쪽 선택');axs[0].set_xlabel('X (m)');axs[0].set_ylabel('Y (m)')
c1=json.loads((P/'input_1.json').read_text())['cycles'];c=next(c for c in c1 if c['running'] and not c['rec_valid']);g=np.frombuffer(gzip.decompress((P/'grids.bin.gz').read_bytes()),dtype=np.int8).reshape(-1,128,128)[c['grid_index']];axs[1].imshow(g,origin='lower',extent=(0,40,-20,20),cmap='gray_r',vmin=0,vmax=100,interpolation='nearest');yaw=c['yaw'];rot=np.array([[np.cos(yaw),-np.sin(yaw)],[np.sin(yaw),np.cos(yaw)]]);a=(route-np.array([c['x'],c['y']]))@rot;axs[1].plot(*a.T,'--o',color='#dd9922',label='지정 경로 (차량 좌표)');a=(np.array(c['rec_preview'])-np.array([c['x'],c['y']]))@rot;axs[1].plot(*a.T,'o-',color='#3399cc',label='참조');axs[1].set_xlim(0,40);axs[1].set_ylim(-20,20);axs[1].legend(fontsize=8);axs[1].set_title('1회 최초 계획 소실: 지도 유효 영역 희박');axs[1].set_xlabel('차량 전방 (m)');axs[1].set_ylabel('차량 왼쪽 (m)');fig.savefig(H/'failure_detail.png',dpi=150);plt.close(fig)
print(checks)
