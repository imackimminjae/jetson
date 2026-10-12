"""Which grid x-origin makes the evening BEV consistent with the world?
Reference map: morning scenario3 BEV frames (received origin 0.00, used as-is), accumulated in world coords.
Test: evening frames projected with x-origin = 0.00 (current override) .. 2.95 (received) .. 5.9, score agreement.
Pose: /motive/vehicle/odom_map (orientation.z = yaw rad), nearest sample before the grid receive time.
Grid cell (col i, row j) -> body (ox + res*(i+.5), -20 + res*(j+.5)) -> world."""
import pickle, numpy as np, sys
RES = 0.3125; W = 128
import os
REF = os.environ.get('REF','045314,050627').split(',')          # morning, origin 0.00, J2 success
TEST = sys.argv[1:] or ['180803', '181932']
WRES = 0.5

def load(tag, step):
    d = pickle.load(open(f'/home/imac/ros2_ws/drive_debug_{tag if "_" in tag else "20261006_"+tag}_light/analysis/decoded.pkl', 'rb'))
    od = d['/motive/vehicle/odom_map']; ot = np.array([o['t'] for o in od])
    xy = np.array([o['xy'][:2] for o in od]); yaw = np.array([o['q'][2] for o in od])
    v = np.array([np.hypot(*o['v'][:2]) for o in od])
    out = []
    for g in d['/bev/occupancy_grid'][::step]:
        i = np.searchsorted(ot, g['t']) - 1
        if i < 0 or v[i] < 2.0: continue          # moving frames only
        out.append((xy[i], yaw[i], np.asarray(g['grid']).reshape(W, W)))
    return out

ii, jj = np.meshgrid(np.arange(W), np.arange(W))   # ii = column (x), jj = row (y)
def world_cells(frames, ox):
    pts, vals = [], []
    for p, a, g in frames:
        bx = ox + RES * (ii + .5); by = -20 + RES * (jj + .5)
        c, s = np.cos(a), np.sin(a)
        wx = p[0] + c * bx - s * by; wy = p[1] + s * bx + c * by
        m = g != -1
        pts.append(np.stack([wx[m], wy[m]], 1)); vals.append(g[m] == 0)
    return np.concatenate(pts), np.concatenate(vals)

def raster(pts, free):
    k = np.floor(pts / WRES).astype(np.int64)
    key = k[:, 0] * 100000 + k[:, 1]
    u, inv = np.unique(key, return_inverse=True)
    f = np.bincount(inv, weights=free.astype(float)); n = np.bincount(inv)
    return dict(zip(u.tolist(), (f / n).tolist()))

ref = []
for t in REF: ref += load(t, 3)
R = raster(*world_cells(ref, 0.0))
Rk = np.array(list(R.keys())); Rv = np.array(list(R.values()))
Rfree = {k for k, v in R.items() if v > 0.8}; Rocc = {k for k, v in R.items() if v < 0.2}
print(f'reference cells={len(R)} free={len(Rfree)} occ={len(Rocc)}')
for t in TEST:
    fr = load(t, 3)
    print(f'== {t}: {len(fr)} moving frames')
    for ox in (-2.0,-1.0,0.0,1.0,2.0,2.95,4.0,5.0,6.0,7.0,8.0):
        T = raster(*world_cells(fr, ox))
        tf = {k for k, v in T.items() if v > 0.8}; to = {k for k, v in T.items() if v < 0.2}
        both = (tf | to) & (Rfree | Rocc)
        agree = len(tf & Rfree) + len(to & Rocc)
        iou = len(tf & Rfree) / max(1, len((tf | Rfree) & (set(T) & set(R))))
        print(f'   x_origin={ox:5.2f}  overlap cells={len(both):6d}  agreement={agree / max(1, len(both)):.4f}  free IoU={iou:.4f}')
