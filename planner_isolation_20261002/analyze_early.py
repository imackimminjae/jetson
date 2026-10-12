from pathlib import Path
from datetime import datetime
import json, pickle
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager

p = Path(__file__).resolve().parent
bag = pickle.load(open(p.parent/'drive_debug_20261001_184521_light/analysis/decoded.pkl', 'rb'))
features = json.loads((p.parent/'drive_debug_20261001_184521_light/analysis/features.json').read_text())
inputs = json.loads((p/'early_inputs.json').read_text())
read = lambda file: {x['id']: x for x in map(json.loads, (p/file).read_text().splitlines())}
feedback = read('early_feedback_results.jsonl')
states = read('early_state_results.jsonl')
prunes = read('early_results.jsonl')
lo = np.array([x['data'] for x in bag['/debug/lower_mpc_trace']])
epoch = datetime.fromisoformat('2026-10-01T18:45:00').timestamp()
clock = lambda t: datetime.fromtimestamp(t).strftime('%H:%M:%S.%f')[:-3]

summary = {'windows': [], 'feedback': {}, 'state_isolation': {}, 'waypoint': {}}
for start, end in [(22.85, 27), (27, 34.474), (34.474, 39.874)]:
    mask = (lo[:, 0] >= epoch+start) & (lo[:, 0] < epoch+end) & (lo[:, 10] == 1)
    rows = [r for r in features if r['valid'] and epoch+start <= r['t'] < epoch+end]
    summary['windows'].append({
        'start': clock(epoch+start), 'end_exclusive': clock(epoch+end),
        'steer_min_max_deg': np.degrees([lo[mask, 26].min(), lo[mask, 26].max()]).tolist(),
        'saturated_cycles': int(np.sum(np.abs(lo[mask, 26]) > .299)),
        'valid_lower_cycles': int(mask.sum()),
        'max_abs_heading_error_deg': float(np.degrees(abs(lo[mask, 7])).max()),
        'first_plan_heading_range_deg': [min(r['h1_deg'] for r in rows), max(r['h1_deg'] for r in rows)],
    })

def stats(result):
    trials = [x['trials'][0] for x in result['series']]
    h = np.array([x.get('h1_deg', np.nan) for x in trials])
    tail = h[-10:][np.isfinite(h[-10:])]
    dh = abs(np.diff(h)); dh = dh[np.isfinite(dh)]
    return {'valid': sum(x['valid'] for x in trials), 'total': len(trials),
            'max_consecutive_heading_change_deg': float(dh.max()) if len(dh) else None,
            'last10_success_heading_span_deg': float(np.ptp(tail)) if len(tail) else None,
            'h1_deg': [float(x) if np.isfinite(x) else None for x in h],
            'branch_steps': [x['branch_step'] for x in trials]}

for key, value in feedback.items():
    if 'series' in value:
        summary['feedback'][key] = stats(value)
    else:
        summary['waypoint'][key] = value['trials'][0]['h1_deg']
for key, value in states.items():
    summary['state_isolation'][key] = stats(value)
summary['prune_changes_deg'] = {
    str(i): prunes[f'prune_early_{i}']['trials'][0]['h1_deg'] - features[i]['h1_deg']
    for i in inputs['selected']}
summary['wp57_logged_jump_deg'] = features[57]['h1_deg'] - features[56]['h1_deg']
summary['wp57_route_turn_before_after_deg'] = [float(np.degrees(inputs['snapshots'][i]['trace'][13])) for i in [56, 57]]
(p/'early_summary.json').write_text(json.dumps(summary, indent=2, ensure_ascii=False, allow_nan=False))

font_manager.fontManager.addfont('/usr/share/fonts/truetype/nanum/NanumGothic.ttf')
plt.rcParams.update({'font.family': 'NanumGothic', 'axes.unicode_minus': False, 'font.size': 10})
fig = plt.figure(figsize=(14, 11), layout='constrained')
gs = fig.add_gridspec(3, 2)
ax1 = fig.add_subplot(gs[0, :]); ax2 = fig.add_subplot(gs[1, :], sharex=ax1)
events = [x['data'] for x in bag['/debug/upper_branch_event']]
good = [(e, f) for e, f in zip(events, features) if e['valid_plan'] and e['t_emit'] < epoch+39.874]
ax1.step([e['t_emit']-epoch for e, f in good], [f['h1_deg'] for e, f in good], where='post', color='#177dc0', label='상위 첫 계획 방향')
mask = (lo[:, 0] < epoch+39.874) & (lo[:, 10] == 1)
ax1.plot(lo[mask, 0]-epoch, np.degrees(lo[mask, 3]), color='#e68c29', label='실제 차체 방향')
ax1.set_ylabel('지도 좌표 방향 (deg)'); ax1.legend(loc='upper left')
ax1.set_title('문제는 40초 이전에도 발생: 계획 방향 반전과 차량의 뒤따르는 회전')
ax2.plot(lo[mask, 0]-epoch, np.degrees(lo[mask, 26]), color='#d34e67', label='조향 출력')
for sign in [-1, 1]: ax2.axhline(sign*np.degrees(.3), color='#777', linestyle='--', linewidth=1)
ax2.set_ylabel('조향각 (deg)'); ax2.set_xlabel('18:45:xx — 시각의 초'); ax2.legend(loc='upper left')
ax2.set_title('34~37초에는 조향 출력이 양쪽 한계 ±17.2°에 도달')
for ax in [ax1, ax2]:
    ax.axvspan(34.474, 37.5, color='#e6aa59', alpha=.1)
    ax.axvline(34.474, color='#555', linestyle=':', linewidth=1.5)
    ax.set_xlim(22.85, 39.87); ax.grid(alpha=.2)
ax1.annotate('경유점 전환: 한 주기 +21.5°', xy=(34.474, -25.95), xytext=(29.5, -12), arrowprops={'arrowstyle':'->'}, fontsize=10)

ax = fig.add_subplot(gs[2, 0])
for key, label, color in [
    ('fixed_B_62', '이전 폭 상태 고정, 이전 계획 재사용', '#177dc0'),
    ('fixed_previous_62', '이전 계획 고정, 폭 상태 갱신', '#e28e29')]:
    data = summary['state_isolation'][key]
    ax.plot(range(1, 41), data['h1_deg'], '-o', ms=2.5, color=color, label=label)
ax.set_title('35.474초 장면 고정: 이전 계획 재사용으로 진동 지속')
ax.set_xlabel('상위 반복 계산 횟수'); ax.set_ylabel('첫 계획 방향 (deg)'); ax.legend(fontsize=9); ax.grid(alpha=.2)
ax = fig.add_subplot(gs[2, 1])
keys = ['wp_56_at57_0.6', 'wp_57_at57_0.6', 'wp_57_at57_0.0']
values = [summary['waypoint'][k] for k in keys]
bars = ax.bar(['전환 전 경유점 유지', '전환 후 경유점\n현재 비용', '전환 후 경유점\n회전 비용 0 (진단용)'], values, color=['#e5a145', '#d85667', '#888'])
ax.bar_label(bars, fmt='%.2f°', padding=3); ax.set_ylim(-48, 0)
ax.set_title('34.474초 같은 장면: 경유점만 바꾸면 14.8° 차이')
ax.set_ylabel('첫 계획 방향 (deg)'); ax.grid(axis='y', alpha=.2)
fig.suptitle('앞쪽 구간 추가 분석 — 작은 도로 후보 하나만의 문제가 아님', fontsize=16)
fig.supxlabel('위 두 그래프: 실제 로그. 아래 두 그래프: 차량을 구동하지 않은 오프라인 원인 분리 시험.', fontsize=10)
fig.savefig(p/'early_oscillation.png', dpi=150)
print(json.dumps({k: summary[k] for k in ['windows', 'waypoint', 'wp57_logged_jump_deg', 'wp57_route_turn_before_after_deg']}, indent=2, ensure_ascii=False))
