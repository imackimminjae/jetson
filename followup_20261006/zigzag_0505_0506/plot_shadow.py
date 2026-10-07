from pathlib import Path
from datetime import datetime
from zoneinfo import ZoneInfo
import json,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager,dates as mdates
p=Path(__file__).resolve().parent;tz=ZoneInfo('Asia/Seoul')
font_manager.fontManager.addfont('/usr/share/fonts/truetype/nanum/NanumGothic.ttf')
plt.rcParams.update({'font.family':'NanumGothic','axes.unicode_minus':False})
rows=json.loads((p/'probe_results.json').read_text())['050627']
fmt=lambda t:datetime.fromtimestamp(t,tz).strftime('%H:%M:%S.%f')[:-3]
rows=[r for r in rows if '05:06:46'<=fmt(r['t'])<'05:06:50']
dates=[datetime.fromtimestamp(r['t'],tz) for r in rows]
fig,axes=plt.subplots(2,1,figsize=(12,7),layout='constrained',sharex=True)
axes[0].plot(dates,np.degrees([r['current']['delta'] for r in rows]),'o-',ms=3,c='#c94c45',label='새 경로로 계산 = 기록된 명령 재현')
axes[0].plot(dates,np.degrees([r['previous']['delta'] for r in rows]),'--',c='#297fac',label='같은 차량 상태·조향 이력에서 직전 경로로 1회 계산')
axes[0].set_ylabel('조향 명령 (°)');axes[0].legend(fontsize=9);axes[0].axhline(0,c='#888',lw=.7)
ev=json.loads((p/'plan_events.json').read_text())['050627'];ev=[e for e in ev if '05:06:46'<=e['time']<'05:06:50' and e['previous_plan_lateral_m']]
axes[1].plot([datetime.fromtimestamp(e['t'],tz) for e in ev],[e['previous_plan_lateral_m'][0] for e in ev],'o-',c='#9857a5',label='k=1 근거리 계획 횡이동')
axes[1].set_ylabel('직전 계획 대비 횡이동 (m)');axes[1].legend(fontsize=9)
for ax in axes:
    ax.grid(alpha=.2)
    for e in ev:ax.axvline(datetime.fromtimestamp(e['t'],tz),c='#aaa',alpha=.4,lw=.6)
axes[1].xaxis.set_major_formatter(mdates.DateFormatter('%H:%M:%S',tz=tz));axes[1].set_xlabel('2026-10-06 KST')
fig.suptitle('경로 변경만으로 조향 방향이 바뀌는 사례\n05:06:48.541: 직전 경로 +8.13° → 새 경로 -3.35° · 점선은 별도 주행 궤적이 아닌 매 시점 1회 계산',fontsize=14)
fig.savefig(p/'path_only_effect.png',dpi=170)
