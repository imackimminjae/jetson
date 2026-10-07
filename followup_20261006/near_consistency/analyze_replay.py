from pathlib import Path
import json
import numpy as np

here = Path(__file__).resolve().parent
summary = {}
def quant(values, q):
    return float(np.quantile(values, q)) if values else None

for tag in ('S1', 'S2', 'R3', 'A'):
    folder = here.parent / ('sc3_0141_0143' if tag.startswith('S') else 'verify')
    cycles = json.loads((folder / f'replay_{tag}.json').read_text())['cycles']
    outputs = json.loads((here / f'replay_{tag}_out.json').read_text())
    old = json.loads((folder / f'replay_{tag}_out.json').read_text())
    baseline = old.get('0_-1.000000')
    # Use distance travelled to compare the same recorded-pose window.
    xy = np.array([[c['x'], c['y']] for c in cycles])
    progress = np.r_[0, np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1))]
    for key, rows in outputs.items():
        near_change, heading_change, adjacent, endpoints, steps = [], [], [], [], []
        failures = 0
        for i, r in enumerate(rows):
            # First wide junction, ending before the final left corner.
            include = 35 <= progress[i] <= 110 if tag.startswith('S') else True
            if not include:
                continue
            if not r['valid']:
                failures += 1
                continue
            p = np.array(r['planned'])
            ref = np.array(r['reference'])
            if r['used_previous'] and len(p) >= 4:
                near_change.append(float(np.linalg.norm(p[1:3] - ref[1:3], axis=1).max()))
                a, b = p[1] - p[0], ref[1] - ref[0]
                heading_change.append(float(abs(np.degrees(np.arctan2(np.cross(b, a), b @ a)))))
            if i and rows[i-1]['valid']:
                prev = np.array(rows[i-1]['planned'])
                a, b = p[1]-p[0], prev[1]-prev[0]
                adjacent.append(float(abs(np.degrees(np.arctan2(np.cross(b, a), b @ a)))))
            h = np.unwrap(np.arctan2(np.diff(p[:,1]), np.diff(p[:,0])))
            steps.extend(np.abs(np.degrees(np.diff(h))).tolist())
        item = dict(window='recorded travel 35..110m' if tag.startswith('S') else 'all',
                    failures=failures, all_failures=sum(not r['valid'] for r in rows),
                    near_displacement_p95_m=quant(near_change,.95),
                    matched_heading_p95_deg=quant(heading_change,.95),
                    consecutive_heading_p95_deg=quant(adjacent,.95),
                    max_segment_turn_deg=max(steps) if steps else None)
        if key == '0_0.000000' and baseline:
            item['matches_existing_baseline'] = all(
                a['valid'] == b['valid'] and (not a['valid'] or
                np.allclose(a['planned'], b['planned'], atol=1e-10, rtol=0))
                for a,b in zip(rows,baseline)) and len(rows)==len(baseline)
        summary[tag+'_'+key] = item
        print(tag, key, json.dumps(item))
(here/'replay_summary.json').write_text(json.dumps(summary, indent=2)+'\n')
