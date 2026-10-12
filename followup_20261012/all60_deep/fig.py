import pickle,json,gzip,numpy as np,matplotlib;matplotlib.use('Agg');import matplotlib.pyplot as plt
from matplotlib import font_manager
font_manager.fontManager.addfont('/usr/share/fonts/truetype/nanum/NanumGothic.ttf');plt.rcParams.update({'font.family':'NanumGothic','axes.unicode_minus':False})
TAUS=np.arange(0,0.61,0.05)
m08=[0.124,0.119,0.116,0.118,0.124,0.136,0.154,0.177,0.205,0.239,0.277,0.320,0.367];m09=[0.323,0.284,0.232,0.188,0.152,0.124,0.105,0.096,0.094,0.102,0.118,0.144,0.180]
D=pickle.load(open('decoded.pkl','rb'));A=pickle.load(open('attempts.pkl','rb'));es=[e for t,e in D['/debug/upper_branch_event']]
H='/home/imac/ros2_ws/followup_20261008/batches_182344_183952_185426_review/scenario3/';E2=json.load(gzip.open(H+'events.json.gz','rt'));S=json.load(open(H+'summary.json'))
BC='/home/imac/ros2_ws/followup_20261009/s1_block_compare/regress/s3/';mm=json.load(open(BC+'static_map.json'));B=np.fromfile(BC+'static_map.bin',dtype=np.int8).reshape(mm['height'],mm['width']);ext=(mm['origin'][0],mm['origin'][0]+mm['width']*mm['res'],mm['origin'][1],mm['origin'][1]+mm['height']*mm['res'])
fig,axs=plt.subplots(1,3,figsize=(21,6.5),gridspec_kw=dict(width_ratios=[1,1.1,1.1]))
ax=axs[0];ax.plot(TAUS,m08,'o-',c='teal',label='10-08 (BEV 40 Hz)');ax.plot(TAUS,m09,'o-',c='crimson',label='10-09 (BEV 21~25 Hz)')
ax.axvline(0.10,c='teal',ls=':');ax.axvline(0.40,c='crimson',ls=':');ax.set_xlabel('BEV를 τ초 전 자세로 놓았을 때 τ (s)');ax.set_ylabel('도로 데이터와의 불일치 (평균 거리, m)');ax.legend();ax.grid(alpha=.3)
ax.set_title('BEV 자세 지연 추정: 10-08 ≈0.10 s → 10-09 ≈0.40 s')
def draw(ax,ev,title):
    ax.imshow(B==0,origin='lower',extent=ext,cmap='Greys_r',alpha=.5)
    for w in ev['intervals_world']:
        k,ci,x0,y0,x1,y1,raw=w[:7];ax.plot([x0,x1],[y0,y1],'-',lw=3,c='#2a7');ax.text((x0+x1)/2+.8,(y0+y1)/2,f"k{int(k)}:{raw:.1f}",fontsize=8,color='#163')
    pv=np.array(ev['preview_world']);ax.plot(pv[:,0],pv[:,1],'--o',c='#555',ms=3,lw=1,label='참조(이전 계획 직선 연장)')
    if ev['published_world']: P=np.array(ev['published_world']);ax.plot(P[:,0],P[:,1],'-',c='crimson',lw=2,label='발행 계획')
    ax.plot(ev['x'],ev['y'],'k^',ms=9);ax.set_xlim(232,285);ax.set_ylim(118,170);ax.set_aspect('equal');ax.grid(alpha=.25);ax.set_title(title,fontsize=11);ax.legend(fontsize=8,loc='upper right')
a08=S['attempts'][0];e08=min([e for e in E2 if a08['start']<=e['t_start']<a08['end']],key=lambda e:abs(e['x']-240.1)+5*(e['y']>150))
a09=[a for a in A['att'] if a['route']==3][0];e09=min([e for e in es if a09['start']<=e['t_start']<a09['end']],key=lambda e:abs(e['x']-240.1)+5*(e['y']>150))
draw(axs[1],e08,f"10-08 S3 (성공) x≈240, yaw {np.degrees(e08['yaw_rad']):.0f}°\nk6 참조점이 두 갈래 사이 → 두 갈래 모두 후보, 북동 선택")
draw(axs[2],e09,f"10-09 S3 (실패) x≈240, yaw {np.degrees(e09['yaw_rad']):.0f}°\nk6 참조점이 동남 도로 안 → 후보 1개(6.2 m), B 6.2로 하락")
fig.suptitle('10-09 전체 배치 S3·S12 n43 오분기: BEV 지연 증가 → 접근 자세 변화 → 먼 단면이 동남 갈래만 잡음',fontsize=13,y=1.02)
fig.text(0.01,-0.02,'배경=10-08 누적 BEV · 녹색=처리 후 통로(라벨 k:원시폭 m) · 같은 계획·제어 바이너리와 설정, BEV 메시지 형식 동일, 조향 응답·속도·EKF heading 동일(±1°)',fontsize=10)
fig.savefig('summary_s3_lag.png',dpi=85,bbox_inches='tight')
