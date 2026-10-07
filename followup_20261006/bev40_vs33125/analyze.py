from pathlib import Path
import json,numpy as np,hashlib,os
from datetime import datetime
from zoneinfo import ZoneInfo
os.environ['MPLCONFIGDIR']='/tmp/codex_mpl_grid'
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
font_manager.fontManager.addfont('/usr/share/fonts/truetype/nanum/NanumGothic.ttf');plt.rcParams.update({'font.family':'NanumGothic','axes.unicode_minus':False})
h=Path(__file__).resolve().parent
clock=lambda t:datetime.fromtimestamp(t,ZoneInfo('Asia/Seoul')).strftime('%H:%M:%S.%f')[:-3]
S={};data={}
for tag,stem in [('175433','175433_aligned'),('174526','174526')]:
 c=json.loads((h/f'{stem}_input.json').read_text())['cycles'];a=json.loads((h/f'{stem}_full40_result.json').read_text());b=json.loads((h/f'{stem}_mask33125_result.json').read_text());data[tag]=(c,a,b);val=[];vm=[]
 for i,(cc,r) in enumerate(zip(c,a['fixed'])):
  if cc['rec_valid']!=r['valid']:vm.append(i)
  if cc['rec_valid'] and r['valid']:val.append(float(np.max(np.linalg.norm(np.array(cc['rec_planned'])-r['planned'],axis=1))))
 S[tag]={'validation':{'cycles':len(c),'validity_mismatches':vm,'valid_plans':len(val),'within_1mm':sum(v<.001 for v in val),'max_plan_error_m':max(val)},'modes':{}}
 for mode in ['fixed','carried']:
  dif=[];changes=[];shapes=[];flags=[];k1=[];intervals=[]
  for i,(cc,x,y) in enumerate(zip(c,a[mode],b[mode])):
   if x['valid']!=y['valid']:flags.append(i)
   if x['intervals']!=y['intervals'] or x['counts']!=y['counts']:intervals.append(i)
   if x['valid'] and y['valid']:
    pp=np.array(x['planned']);qq=np.array(y['planned'])
    if pp.shape!=qq.shape:shapes.append(i);continue
    dd=np.linalg.norm(pp-qq,axis=1);dif.append(float(max(dd)));k1.append(float(dd[1]))
    if max(dd)>.001:changes.append({'i':i,'time':clock(cc['t']),'max_m':float(max(dd)),'k1_m':float(dd[1])})
  focus=[i for i,cc in enumerate(c) if 232<cc['x']<260];branch={}
  for label,rows in [('full40',a[mode]),('mask33125',b[mode])]:
   branch[label]=[{'time':clock(c[i]['t']),'end':rows[i].get('planned',[None])[-1]} for i in focus]
  S[tag]['modes'][mode]={'cycles':len(c),'failures_full':sum(not r['valid'] for r in a[mode]),'failures_mask':sum(not r['valid'] for r in b[mode]),'validity_differences':flags,'horizon_shape_differences':shapes,'max_plan_difference_m':max(dif,default=0),'max_k1_difference_m':max(k1,default=0),'changed_over_1mm':changes,'interval_changed_cycles':intervals,'branch':branch}
(h/'summary.json').write_text(json.dumps(S,ensure_ascii=False,indent=2));print(json.dumps({t:{'validation':v['validation'],'modes':{m:{k:z for k,z in q.items() if k!='branch'} for m,q in v['modes'].items()}} for t,v in S.items()},ensure_ascii=False,indent=2))
# Confirm source files and source bags remained unchanged.
manifest=json.loads((h/'sources_before_sha256.json').read_text());changed=[p for p,sha in manifest.items() if hashlib.sha256(Path(p).read_bytes()).hexdigest()!=sha];(h/'source_integrity.json').write_text(json.dumps({'checked':len(manifest),'changed':changed,'note':'Live config changed externally during analysis; saved bag config copies used by all replay runs. No restore performed.'},indent=2)); assert all(p.endswith('/config/tracking_control_split.yaml') for p in changed),changed
fig,axs=plt.subplots(1,2,figsize=(12,5),layout='constrained')
c,a,b=data['175433'];gp=json.loads((h/'175433_input.json').read_text())['route'];ax=axs[0];ax.plot(*np.array(gp).T,'k--o',ms=3,label='지정 경로')
focus=[i for i,cc in enumerate(c) if 235<cc['x']<253]
for n,i in enumerate(focus[::2]):
 for rows,color,ls,label in [(a['carried'],'#276ca5','-','40m'),(b['carried'],'#e68126','--','33.125m 이후 가림')]:
  pp=np.array(rows[i]['planned']);ax.plot(*pp.T,color=color,ls=ls,lw=2,label=label if n==0 else None)
ax.set_xlim(230,284);ax.set_ylim(123,174);ax.set_aspect('equal');ax.set_title('최신17:54 · 두 조건의 계획이 완전히 겹침');ax.legend(fontsize=8);ax.set_xlabel('지도 X (m)');ax.set_ylabel('지도 Y (m)');ax.grid(alpha=.2)
c,a,b=data['174526'];i=25;ax=axs[1];ref=np.array(a['fixed'][i]['reference']);ax.plot(*ref.T,':',c='gray',label='동일 참조')
for rows,color,ls,label in [(a['fixed'],'#276ca5','-','40m'),(b['fixed'],'#e68126','--','33.125m 이후 가림')]:
 pp=np.array(rows[i]['planned']);ax.plot(*pp.T,'o',c=color,ms=3);ax.plot(*pp.T,color=color,ls=ls,lw=2,label=label)
ax.set_aspect('equal');ax.set_title('직전17:46:17.441 · 원거리 계획 차이 최대3.75m\nk1과 B는 같음, 마지막 오분기는 두 조건 모두 유지');ax.set_xlabel('지도 X (m)');ax.set_ylabel('지도 Y (m)');ax.legend(fontsize=8);ax.grid(alpha=.2)
fig.suptitle('원본40m vs 끝22열만 미관측 처리 · 좌표/해상도/Nmax7 유지',fontsize=14);fig.savefig(h/'comparison.png',dpi=150);plt.close(fig)
