import sys;sys.path.insert(0,'/home/imac/ros2_ws/followup_20261009/edge_cases')
from common import *
H='/home/imac/ros2_ws/followup_20261008/batches_182344_183952_185426_review/scenario3/'
es=json.load(gzip.open(H+'events.json.gz','rt'));S=json.load(open(H+'summary.json'));A={a['attempt']:a for a in S['attempts']}
def pick(i,x):
    a=A[i];c=[e for e in es if a['start']<=e['t_start']<a['end'] and e['y']<150];return min(c,key=lambda e:abs(e['x']-x))
fig,axs=plt.subplots(1,2,figsize=(15,7.5))
for ax,(i,x,title) in zip(axs,[(2,251.3,'2회 (성공): x=245에서 y≈139.7, yaw -5°'),(4,253.7,'4회 (QP 불가): x=245에서 y≈141.2 (안쪽), yaw -7°')]):
    bg(ax,'regress/s3/');e=pick(i,x)
    for w in e['intervals_world']:
        k,ci,x0,y0,x1,y1,raw=w[:7];ax.plot([x0,x1],[y0,y1],'-',lw=3,c='#2a7');ax.text((x0+x1)/2+1,(y0+y1)/2,f"k{int(k)}:{raw:.1f}",fontsize=9,color='#163')
    pv=np.array(e['preview_world']);ax.plot(pv[:,0],pv[:,1],'--o',c='#555',ms=3,lw=1,label='참조')
    if e['published_world']: P=np.array(e['published_world']);ax.plot(P[:,0],P[:,1],'-',c='crimson',lw=2,label='발행 계획')
    ax.plot(e['x'],e['y'],'k^',ms=9,label='차량');ax.set_title(f"{title}\n이 주기: {'성공' if e['valid_plan'] else e['status'][:34]}",fontsize=11)
    ax.set_xlim(238,275);ax.set_ylim(128,162);ax.set_aspect('equal');ax.grid(alpha=.25)
axs[0].legend(loc='lower right',fontsize=9)
fig.suptitle('사례 4  시나리오3 마지막 76° 코너: 안쪽으로 접근하면 북동 갈래 단면이 짧게 잘려 QP 불가',fontsize=13)
note(fig,'실제 4·5·7회: (254~257,140)에서 2주기 연속 실패 후 회복, 10/10 완료 · 같은 단계 원시폭 성공 8.4~9.3 m vs 실패 3.5~4.6 m · 배경=10회 누적 BEV')
fig.savefig('case04_s3_corner_truncation.png',dpi=90,bbox_inches='tight');plt.close(fig)
# ---- case 7 + 10 from closed-loop sim results (scenario1)
R=[json.loads(l) for l in open(BC+'cl_results.jsonl')]
fig,axs=plt.subplots(1,2,figsize=(15,7.5))
for ax,v in zip(axs,['base','confirm+recover3']):
    bg(ax);n=0
    for r in R:
        if r['config']['name']!=v or r['plant']['name']!='fit_tau0.74_d0.1': continue
        x=[s['x'] for s in r['states']];y=[s['y'] for s in r['states']];wrong=r['outcome']=='north_branch';n+=wrong
        ax.plot(x,y,c='crimson' if wrong else 'teal',lw=1.6 if wrong else .7,alpha=.9)
    ax.set_xlim(100,170);ax.set_ylim(-35,15);ax.set_aspect('equal');ax.grid(alpha=.25);ax.set_title(f"{'현재 규칙' if v=='base' else 'confirm(B 감소는 2주기 연속일 때만)+recover3'}: 북쪽 오분기 {n}/25",fontsize=11)
fig.suptitle('사례 7  B 감소를 늦추면 진입부 중앙(섬 끝) 겨냥이 길어져 북쪽 갈래로 빠짐 (폐루프 시뮬)',fontsize=13)
note(fig,'폐루프 시뮬: 운영 상·하위 코드, 조향 τ0.74 s·지연0.1 s, 시작점5×위상5 · 빨강=북쪽 오분기')
fig.savefig('case07_confirm_side_effect.png',dpi=90,bbox_inches='tight');plt.close(fig)
# ---- case 10: phase sensitivity grid (start x phase) for base, scenario1 oscillation and scenario3 corner outcome
R3=[json.loads(l) for l in open(BC+'regress/s3/cl_results.jsonl')]
fig,axs=plt.subplots(1,2,figsize=(13,5))
for ax,(RR,title,bad) in zip(axs,[(R,'시나리오1 두 번째 분기 후 진동(조향 표준편차>9°)·오분기',lambda r:r['outcome']!='east_done' or np.std([np.degrees(s['cmd']) for s in r['states'] if s['x']>=120 and s['y']>-20] or [0])>9),(R3,'시나리오3 마지막 코너 정지',lambda r:r['outcome']!='goal')]):
    G=np.full((5,5),np.nan);names=[]
    for r in RR:
        if r['config']['name']!='base' or r['plant']['name']!='fit_tau0.74_d0.1': continue
        sn=r['start']['name'];names.append(sn) if sn not in names else None;G[names.index(sn),r['config']['phase']]=bad(r)
    ax.imshow(G,cmap='RdYlGn_r',vmin=0,vmax=1);ax.set_xticks(range(5));ax.set_xticklabels([f'위상{p}' for p in range(5)]);ax.set_yticks(range(len(names)));ax.set_yticklabels([f'시작 {n}' for n in names])
    for i in range(5):
        for j in range(5): ax.text(j,i,'실패' if G[i,j]==1 else '성공',ha='center',va='center',fontsize=9)
    ax.set_title(f'{title}\n(현재 규칙, 25회 중 실패 {int(np.nansum(G))})',fontsize=11)
fig.suptitle('사례 10  같은 시작점에서도 상위 2 Hz 계획 시점(위상)만 바뀌면 결과가 갈림',fontsize=13,y=1.06)
note(fig,'폐루프 시뮬 · 위상 = 0.1 s 하위 주기 대비 0.5 s 상위 계획 시점 offset · 단일 위상 결과로 성패를 판단하면 안 됨')
fig.savefig('case10_phase_sensitivity.png',dpi=90,bbox_inches='tight');plt.close(fig)
print('B done')
