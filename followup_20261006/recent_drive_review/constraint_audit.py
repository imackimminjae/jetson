"""Audit the observed one-candidate constraint contraction, without rerunning a solver."""
from pathlib import Path
import pickle,json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from datetime import datetime
from zoneinfo import ZoneInfo
HERE=Path(__file__).resolve().parent
series=pickle.load((HERE/'series.pkl').open('rb'))
font_manager.fontManager.addfont('/usr/share/fonts/truetype/nanum/NanumGothic.ttf')
plt.rcParams.update({'font.family':'NanumGothic','axes.unicode_minus':False})
fig,axes=plt.subplots(1,3,figsize=(15,6),layout='constrained');out=[]
for ax,(tag,when) in zip(axes,[('044438','04:44:48'),('044521','04:45:39'),('044738','04:47:49')]):
    e=next(e for e in series[tag]['events'] if datetime.fromtimestamp(e['t_start'],ZoneInfo('Asia/Seoul')).strftime('%H:%M:%S')==when)
    ref=np.array(e['preview_world']);plan=np.array(e['planned_world']);row={'bag_tag':tag,'time':when,'steps':[]}
    ax.plot(*ref[:4].T,'o--',c='#1874ad',label='이번 주기 입력 참조')
    ax.plot(*plan[:4].T,'o-',c='#d64d4d',label='새 계획')
    for k in [1,2]:
        r=ref[k];items=[]
        for z in e['intervals_world']:
            if z[0]!=k:continue
            a,b=np.array(z[2:4]),np.array(z[4:6]);L=np.linalg.norm(b-a);axis=(b-a)/L;u=np.dot(r-a,axis);ins=(z[6]-z[7])/2
            raw=np.array([a-ins*axis,b+ins*axis]);soft=np.array([a,b])
            ax.plot(*raw.T,c='#b6a230',lw=2,label='축소 전 단면' if k==1 else None)
            ax.plot(*soft.T,c='#329064',lw=5,label='실제 허용 단면' if k==1 else None)
            items.append({'candidate':z[1],'raw_width_m':z[6],'processed_width_m':z[7], 'reference_soft_gap_m':float(max(0.,-u,u-L)), 'reference_raw_gap_m':float(max(0.,-u-ins,u-L-ins)), 'source_mode':z[8],'fallback_reason':z[9]})
        row['steps'].append({'k':k,'candidates':items,'plan_minus_reference_m':float(np.linalg.norm(plan[k]-r))})
    gap=row['steps'][0]['candidates'][0]['reference_soft_gap_m']
    ax.set_title(f'{when} / bag {tag}\nk=1 참조를 최소 {gap:.2f} m 이동해야 허용',fontsize=11)
    ax.scatter(*ref[0],c='k',s=30);ax.set_xlim(214,242);ax.set_ylim(135,159);ax.set_aspect('equal');ax.grid(alpha=.2);ax.set_xlabel('지도 X (m)');ax.set_ylabel('지도 Y (m)');out.append(row)
axes[0].legend(fontsize=9)
fig.suptitle('진행 85~88 m: 후보가 하나여도 근거리 계획이 크게 이동\n참조는 축소 전 단면 안에 있으나 실제 허용 단면 밖에 있음 · 이 이동이 마지막 오분기의 단독 원인인지는 미검증',fontsize=14)
fig.savefig(HERE/'constraint_contraction.png',dpi=170)
(HERE/'constraint_audit.json').write_text(json.dumps(out,indent=2)+'\n')
