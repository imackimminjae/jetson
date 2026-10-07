#!/usr/bin/env python3
"""Rollback for the 2026-10-06 upper branch model change.

Default (no args): restore only tracking_control_split.yaml. The new code defaults every new flag to
false and the old YAML has upper_r_dpsi 1.5, which was verified to reproduce the previous planner
exactly (172 simulated runs, identical trajectories). No rebuild needed; restart upper_planner_node.

--full: also restore tracking_control.cpp / tracking_control.hpp and rebuild virtual_control.

Refuses to overwrite a file that changed after this change was applied.
"""
import hashlib, shutil, subprocess, sys
from pathlib import Path
RB = Path(__file__).resolve().parent
SRC = Path('/home/imac/ros2_ws/src/virtual_control')
FILES = {'yaml': (SRC / 'config/tracking_control_split.yaml', RB / 'tracking_control_split.yaml.before'),
         'cpp': (SRC / 'src/tracking_control.cpp', RB / 'tracking_control.cpp.before'),
         'hpp': (SRC / 'include/virtual_control/tracking_control.hpp', RB / 'tracking_control.hpp.before')}
after = {line.split()[1]: line.split()[0] for line in (RB / 'after_sha256.txt').read_text().splitlines()}
def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()
keys = ['yaml', 'cpp', 'hpp'] if '--full' in sys.argv else ['yaml']
for k in keys:
    target, _ = FILES[k]
    rel = str(target.relative_to(SRC))
    if sha(target) != after[rel]:
        sys.exit(f'ABORT: {target} changed after the 2026-10-06 change; not overwriting. Compare manually.')
for k in keys:
    target, backup = FILES[k]
    shutil.copy2(backup, target)
    print('restored', target)
if '--full' in sys.argv:
    cmd = 'source /opt/ros/humble/setup.bash && cd /home/imac/ros2_ws && colcon build --symlink-install --packages-select virtual_control --parallel-workers 1 --cmake-args -DBUILD_TESTING=ON'
    subprocess.run(['bash', '-c', cmd], check=True)
print('Done. Restart upper_planner_node. Startup log should no longer show "upper branch model: cumulative_increment=1".')
