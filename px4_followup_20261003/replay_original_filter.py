"""Offline conditional replay: ACK arrival stands in for actuator update time.
No board traffic. ACKs do not prove one-to-one uORB publications.
"""
import json
import pickle
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

root = Path(__file__).resolve().parent
bag = root.parent / 'drive_debug_20261003_051409_light/analysis'
data = pickle.load(open(bag / 'decoded.pkl', 'rb'))
trace = np.array([v['data'] for v in data['/debug/lower_mpc_trace']])
hil = [v for v in json.loads((bag / 'telemetry.json').read_text())
       if v.get('mavpackettype', v.get('type')) == 'HIL_STATE_QUATERNION']
t = np.array([v['t'] for v in hil])
vel = np.array([[v['vx'], v['vy']] for v in hil]) * .01
q = np.array([v['attitude_quaternion'] for v in hil])
yaw = np.arctan2(2*(q[:, 0]*q[:, 3]+q[:, 1]*q[:, 2]), 1-2*(q[:, 2]**2+q[:, 3]**2))
beta = np.arctan2(np.sin(np.arctan2(vel[:, 1], vel[:, 0])-yaw),
                  np.cos(np.arctan2(vel[:, 1], vel[:, 0])-yaw))
actual = -np.arctan(2*np.tan(beta))
ack = json.loads((root / 'command_ack_rate.json').read_text())
at = np.array(ack['samples'])[:, 0]
ct, cmd = trace[:, 0], trace[:, 26]
ac = cmd[np.clip(np.searchsorted(ct, at, side='right')-1, 0, len(cmd)-1)]
filtered = []
x = 0.
for u in ac:
    x += (.0025/.05)*(u-x)
    filtered.append(x)
pred = np.array(filtered)[np.clip(np.searchsorted(at, t, side='right')-1, 0, len(at)-1)]
# Reference: correctly time-integrated tau=0.05 s, ZOH command at 400 Hz.
st = np.arange(at[0], at[-1], .0025)
su = cmd[np.clip(np.searchsorted(ct, st, side='right')-1, 0, len(cmd)-1)]
sx = np.zeros(len(st))
for i in range(1, len(st)):
    sx[i] = sx[i-1] + (-np.expm1(-.0025/.05))*(su[i-1]-sx[i-1])
fast = np.interp(t, st, sx)
valid = (t >= at[0]+1) & (t <= at[-1]) & (np.linalg.norm(vel, axis=1) > 3)
rmse = lambda v: float(np.degrees(np.sqrt(np.mean((v[valid]-actual[valid])**2))))
result = {
    'assumptions': 'SIH 400Hz; filter tau .05; each command ACK represents one actuator update; receive timestamps approximate update time; gain 1; no fitted parameters.',
    'ack_hz': ack['summary']['rate_hz'],
    'equivalent_tau_s': -1/ack['summary']['rate_hz']/np.log(.95),
    'fast_model_rmse_deg': rmse(fast),
    'ack_gated_original_filter_rmse_deg': rmse(pred),
    'count': int(valid.sum()),
    'limitation': 'ACK timing is not actuator_outputs_sim timing; this conditional replay supports but does not prove the firmware cause.'
}
(root / 'replay_result.json').write_text(json.dumps(result, indent=2))
print(json.dumps(result, indent=2))
fig, ax = plt.subplots(figsize=(12, 4))
start = t[valid][0]
ax.step(ct-start, np.degrees(cmd), where='post', color='#c62828', alpha=.55, label='Command')
ax.plot(t[valid]-start, np.degrees(actual[valid]), color='#087e8b', label='SIH equivalent steering')
ax.plot(t[valid]-start, np.degrees(pred[valid]), '--', color='#7b1fa2', label='Original filter replay using ACK events (no fit)')
ax.set(xlim=(0, t[valid][-1]-start), xlabel='Time (s)', ylabel='Steering (deg)',
       title='Conditional replay: 400Hz dt, tau=0.05s, filter updates only on ACK events')
ax.legend(fontsize=8);ax.grid(alpha=.2);fig.tight_layout()
fig.savefig(root / 'original_filter_replay.png', dpi=160)
