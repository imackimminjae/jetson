import sys;sys.path.insert(0,'/home/imac/ros2_ws/followup_20261009/edge_cases')
from common import *
D=pickle.load(open(S1,'rb'));es,cs,od,att=D['es'],D['cs'],D['od'],D['att'];A={a['i']:a for a in att}
def ev(i,t):
    a=A[i];return min(es,key=lambda e:abs(e['t_start']-(a['start']+t)))
def B_at(e): return cs[np.argmin(abs(cs[:,0]-e['t_start']))]
def draw_cycle(ax,e,title,hl=None):
    bg(ax);m=(od[:,0]>e['t_start']-8)&(od[:,0]<e['t_start']+0.1);
    pv=np.array(e['preview_world']);ax.plot(pv[:,0],pv[:,1],'--o',c='#555',ms=3,lw=1,label='참조(이전 계획 연장)')
    for w in e['intervals_world']:
        k,ci,x0,y0,x1,y1,raw=w[:7];ref=ci==0  # proxy; recolor below
        ax.plot([x0,x1],[y0,y1],'-',lw=3,c='#2a7',alpha=.8)
        ax.text((x0+x1)/2,(y0+y1)/2,f"k{int(k)}:{raw:.1f}",fontsize=8,color='#163',ha='center')
    ax.plot(e['x'],e['y'],'k^',ms=9,label='차량')
    if e['published_world']: P=np.array(e['published_world']);ax.plot(P[:,0],P[:,1],'-',c='crimson',lw=2,label='발행 계획')
    r=B_at(e);ax.set_title(f"{title}\nB {r[1]:.1f}→{r[2]:.1f} m, 후보 {r[4]:.1f} m(k{int(r[5]) if r[5]>=0 else -1})",fontsize=11)
    if hl: ax.add_patch(plt.Circle(hl,2.2,fill=False,ec='orange',lw=2))
    ax.set_aspect('equal');ax.grid(alpha=.25)
# ---- case 1+2: reference in gore keeps B~14 ; sliver at gore tip
fig,axs=plt.subplots(1,2,figsize=(15,7.5))
draw_cycle(axs[0],ev(1,27.6),'1회 t=27.6 s: k5·k6 단면이 두 갈래로 갈리고\n참조점은 그 사이(섬 위) → 어느 구간도 참조 미포함',hl=(123.7,-0.3))
draw_cycle(axs[1],ev(5,27.4),'5회 t=27.4 s: k5 동쪽 갈래가 참조를 포함\n→ 7.4 m 후보로 B가 즉시 감소')
for a in axs: a.set_xlim(104,140);a.set_ylim(-28,10)
axs[0].legend(loc='lower right',fontsize=9)
fig.suptitle('사례 1·2  참조점이 갈래 사이 섬에 놓이면 B 후보가 사라짐 (주황 원: 섬 끝 0.1 m 조각 구간)',fontsize=13)
note(fig,'시나리오1 20261008_182344 · 녹색 선=처리 후 통로 구간(라벨 k단계:원시폭 m), 배경=10회 누적 BEV · 1회는 B가 진입부 폭 14~15 m로 유지되어 진입부 중앙(섬 끝)을 겨냥')
fig.savefig('case01_02_gore_reference.png',dpi=90,bbox_inches='tight');plt.close(fig)
# ---- case 3: viewpoint dependent narrow reading
fig,axs=plt.subplots(1,2,figsize=(15,7))
draw_cycle(axs[0],ev(1,31.1),'1회 t=31.1 s: 차량 y≈-6.7(위쪽으로 붙음)\nk6 단면 (149,-6) 원시폭 2.0 m',hl=(149.0,-6.1))
draw_cycle(axs[1],ev(5,30.4),'5회 t=30.4 s: 차량 y≈-11.4\n같은 부근 k6 단면 원시폭 3.7 m',hl=(147.4,-4.5))
for a in axs: a.set_xlim(115,155);a.set_ylim(-18,8)
fig.suptitle('사례 3  같은 지점이 차량 위치(시점)에 따라 다른 폭으로 읽힘',fontsize=13)
note(fig,'x≈147~149 부근 BEV 도로 위쪽 경계가 흐림 · 8회 중 차량 y≈-7 시 1.7~2.3 m, y≈-9~-11.5 시 2.9~4.1 m')
fig.savefig('case03_viewpoint_narrow.png',dpi=90,bbox_inches='tight');plt.close(fig)
# ---- case 6: B lock timeline vs distance
fig,ax=plt.subplots(figsize=(13,5.5))
for a in att:
    m=(od[:,0]>=a['start'])&(od[:,0]<a['end']);O=od[m];s=np.concatenate([[0],np.cumsum(np.hypot(np.diff(O[:,1]),np.diff(O[:,2])))])
    c=cs[(cs[:,0]>=a['start'])&(cs[:,0]<a['end'])];col='crimson' if a['i'] in (1,3,8) else ('#999' if a['i'] in (2,7,9) else 'teal')
    ax.plot(np.interp(c[:,0],O[:,0],s),c[:,2],drawstyle='steps-post',c=col,lw=1.6 if col!='#999' else 1,label={1:'흔들린 1·3·8회',5:'부드러운 4·5·6·10회',2:'중간/오분기 2·7·9회'}.get(a['i']))
ax.axhspan(0,3,color='crimson',alpha=.06);ax.text(150,2.3,'B≈2 m 고정: 정상 도로(5.6 m) 후보가 1.5×B=3 m를 넘어 계속 거부',color='crimson')
ax.set_xlim(100,200);ax.set_xlabel('주행 거리 (m)');ax.set_ylabel('B (m)');ax.grid(alpha=.3);ax.legend()
ax.set_title('사례 6  B 고정: 한 번 좁게 읽힌 값이 "1.5×B 초과 후보 거부" 규칙으로 회복되지 않음 (시나리오1 10회)')
note(fig,'관련: 이전 시나리오2 17:17 7회 — 0.4 m 조각 구간으로 B 6.1→0.4 m, 이후 6.4·7.8·8.4 m 후보 모두 거부')
fig.savefig('case06_B_lock.png',dpi=90,bbox_inches='tight');plt.close(fig)
print('A done')
