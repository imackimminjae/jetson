#!/usr/bin/env python3
"""Plot recorded EKF trajectories and applied steering commands for C6."""
import argparse
import csv
import json
import os
from pathlib import Path

os.environ.setdefault('MPLCONFIGDIR', '/tmp/c6_comparison_matplotlib')
os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
from matplotlib.lines import Line2D
import rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray, String
import yaml

COLORS = {'state2_no_tau': '#D75B32', 'state3_tau074': '#1878B5'}
LABELS = {'state2_no_tau': '2-state · 시정수 제외', 'state3_tau074': '3-state · τ=0.74초'}


def extract(root, output):
    plan = json.loads((root/'plan.json').read_text())
    progress = json.loads((root/'progress.json').read_text())
    records = []
    for job in plan['jobs']:
        session = Path(progress['completed'][job['id']])
        summary, = session.glob('results/*/summary.csv')
        with summary.open() as stream:
            attempts = list(csv.DictReader(stream))
        config = yaml.safe_load((session/'effective_config.yaml').read_text())
        limit = config['lower_tracking_mpc_node']['ros__parameters']['lower_max_steering_angle_rad']
        route = np.loadtxt(session/'routes'/f"{job['scenario']}.csv", delimiter=',', skiprows=1)[:, :2]
        phases, poses, traces = {}, [], []
        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=str(session/'bag'), storage_id='mcap'),
                    rosbag2_py.ConverterOptions('cdr', 'cdr'))
        reader.set_filter(rosbag2_py.StorageFilter(topics=[
            '/knu_batch/status', '/motive/vehicle/odom_map', '/debug/lower_mpc_trace']))
        while reader.has_next():
            topic, raw, stamp = reader.read_next()
            if topic == '/knu_batch/status':
                status = json.loads(deserialize_message(raw, String).data)
                phases[status['sequence']] = status
            elif topic == '/motive/vehicle/odom_map':
                msg = deserialize_message(raw, Odometry)
                t = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
                if t == 0:
                    t = stamp*1e-9
                poses.append([t, msg.pose.pose.position.x, msg.pose.pose.position.y])
            else:
                traces.append(list(deserialize_message(raw, Float64MultiArray).data))
        del reader
        poses, traces = np.asarray(poses), np.asarray(traces)
        phases = sorted(phases.values(), key=lambda s: s['phase_stamp_sec'])
        for number, attempt in enumerate(attempts, 1):
            start = next(s['phase_stamp_sec'] for s in phases
                         if s.get('token') == attempt['token'] and s['phase'] == 'running')
            end = next(s['phase_stamp_sec'] for s in phases
                       if s['phase_stamp_sec'] > start and s['phase'] != 'running')
            pose = poses[(poses[:, 0] >= start) & (poses[:, 0] < end)].copy()
            trace = traces[(traces[:, 0] >= start) & (traces[:, 0] < end)].copy()
            if len(pose) < 2 or len(trace) < 2 or not np.isfinite(pose).all():
                raise ValueError(f"Missing trajectory/steering: {job['id']} run {number}")
            assert np.all(trace[:, 21] == float(job['actuator_model_enabled']))
            assert np.isfinite(trace[:, [0, 26]]).all()
            pose[:, 0] -= start
            trace[:, 0] -= start
            record = dict(model=job['set'], scenario=job['scenario'], run=number,
                          outcome=attempt['outcome'], pose=pose, trace=trace,
                          route=route, limit=limit, duration=end-start)
            records.append(record)
            name = f"{job['set']}_{job['scenario']}_run{number}"
            np.savez_compressed(output/'data'/f'{name}.npz',
                                pose_time_xy=pose, trace=trace, route=route)
            np.savetxt(output/'data'/f'{name}_trajectory.csv', pose, delimiter=',',
                       header='elapsed_sec,map_x_m,map_y_m', comments='')
            np.savetxt(output/'data'/f'{name}_steering.csv',
                       np.column_stack([trace[:, 0], trace[:, 26], np.degrees(trace[:, 26]), trace[:, 12]]),
                       delimiter=',', header='elapsed_sec,applied_steering_cmd_rad,applied_steering_cmd_deg,control_mode', comments='')
    assert len(records) == 8
    return records


def main(root):
    output = root/'plots'
    (output/'data').mkdir(parents=True, exist_ok=True)
    font = Path('/usr/share/fonts/truetype/nanum/NanumGothic.ttf')
    if font.exists():
        plt.rcParams['font.family'] = FontProperties(fname=str(font)).get_name()
    plt.rcParams.update({'axes.unicode_minus': False, 'font.size': 11,
                         'axes.spines.top': False, 'axes.spines.right': False,
                         'axes.titlesize': 13, 'axes.labelsize': 11,
                         'savefig.facecolor': 'white', 'pdf.fonttype': 42})
    records = extract(root, output)
    fig, axes = plt.subplots(1, 2, figsize=(11.4, 6.7))
    for ax, scenario in zip(axes, ('scenario1', 'scenario3')):
        selected = [r for r in records if r['scenario'] == scenario]
        route = selected[0]['route']
        ax.plot(route[:, 0], route[:, 1], ':o', color='#444444', lw=1.4,
                ms=4, zorder=1, label='기준 웨이포인트 경로')
        for r in selected:
            x, y = r['pose'][:, 1:].T
            ax.plot(x, y, color=COLORS[r['model']], ls='-' if r['run'] == 1 else '--',
                    lw=1.9, alpha=.9, zorder=3)
            ax.scatter(x[-1], y[-1], marker='x' if r['outcome'] == 'off_route' else 'o',
                       color=COLORS[r['model']], s=65 if r['outcome'] == 'off_route' else 22,
                       zorder=5, linewidths=1.7)
            if r['outcome'] == 'off_route':
                ax.annotate('2-state 2회차 · 경로 이탈', (x[-1], y[-1]),
                            xytext=(-7, -25), textcoords='offset points', ha='right',
                            color=COLORS[r['model']], fontsize=10,
                            arrowprops=dict(arrowstyle='-', color=COLORS[r['model']]))
        ax.scatter(route[0, 0], route[0, 1], marker='s', s=44, facecolor='white',
                   edgecolor='#444444', zorder=6)
        ax.set(title=f"시나리오 {scenario.removeprefix('scenario')}",
               xlabel='지도 X (m)', ylabel='지도 Y (m)')
        ax.set_aspect('equal', adjustable='datalim')
        ax.grid(alpha=.2)
        ax.margins(.12)
    handles = [Line2D([], [], color=COLORS[m], lw=2.3, label=LABELS[m]) for m in COLORS]
    handles += [Line2D([], [], color='#555', ls='-', label='1회차'),
                Line2D([], [], color='#555', ls='--', label='2회차'),
                Line2D([], [], color='#444', ls=':', marker='o', ms=3, label='기준 웨이포인트 경로')]
    fig.legend(handles=handles, loc='upper center', bbox_to_anchor=(.5, .94), ncol=3, frameon=False)
    fig.suptitle('2-state / 3-state 주행 자취 비교', fontsize=18, y=.995)
    fig.text(.5, .018, '지도에 정렬한 EKF 위치 · 각 2회 · 주행 구간만 표시 (종료 후 관성 이동·준비 주행 제외)',
             ha='center', fontsize=10, color='#555')
    fig.subplots_adjust(top=.79, bottom=.13, left=.09, right=.97, wspace=.29)
    for extension in ('png', 'pdf'):
        fig.savefig(output/f'trajectories.{extension}', dpi=190)
    plt.close(fig)

    fig, axes = plt.subplots(2, 2, figsize=(12.6, 7.5), sharey=True)
    for row, scenario in enumerate(('scenario1', 'scenario3')):
        selected = [r for r in records if r['scenario'] == scenario]
        max_time = max(r['duration'] for r in selected)
        for col, run in enumerate((1, 2)):
            ax = axes[row, col]
            for r in [r for r in selected if r['run'] == run]:
                t = r['trace'][:, 0]
                angle = np.degrees(r['trace'][:, 26])
                # Preserve neutral outputs; do not replace them with QP targets.
                ax.plot(t, angle, color=COLORS[r['model']], lw=1.5)
                if r['outcome'] == 'off_route':
                    ax.text(.98, .95, '2-state: 경로 이탈로 종료', transform=ax.transAxes,
                            ha='right', va='top', fontsize=10, color=COLORS[r['model']])
            limit = np.degrees(selected[0]['limit'])
            for sign in (-1, 1):
                ax.axhline(sign*limit, color='#777', lw=.9, ls='--', alpha=.8)
            ax.axhline(0, color='#bbb', lw=.7)
            ax.set(title=f"시나리오 {scenario.removeprefix('scenario')} · {run}회차",
                   xlabel='출발 후 시간 (초)', ylabel='출력 조향 명령 (도)',
                   xlim=(0, max_time+.7), ylim=(-19.5, 19.5))
            ax.set_yticks([-15, -10, -5, 0, 5, 10, 15])
            ax.grid(alpha=.2)
    handles = [Line2D([], [], color=COLORS[m], lw=2.3, label=LABELS[m]) for m in COLORS]
    handles += [Line2D([], [], color='#777', ls='--', label='조향각 제한 ±0.30 rad (±17.2°)')]
    fig.legend(handles=handles, loc='upper center', bbox_to_anchor=(.5, .935), ncol=3, frameon=False)
    fig.suptitle('2-state / 3-state 출력 조향각 비교', fontsize=18, y=.995)
    fig.text(.5, .018, '필터 적용 후 출력 명령 (실측 조향각 아님) · 각 회차 출발 시각을 0초로 정렬 · 중립 출력 구간 포함',
             ha='center', fontsize=10, color='#555')
    fig.subplots_adjust(top=.83, bottom=.11, left=.085, right=.98, hspace=.48, wspace=.18)
    for extension in ('png', 'pdf'):
        fig.savefig(output/f'steering.{extension}', dpi=190)
    plt.close(fig)
    (output/'plot_manifest.json').write_text(json.dumps(dict(
        attempts=8, trajectory_source='/motive/vehicle/odom_map (EKF, map-aligned)',
        steering_source='/debug/lower_mpc_trace[26], applied command in radians, converted to degrees',
        interval='running phase only; each attempt uses its own zero time',
        warmup_included=False, coasting_included=False,
        runs=[dict(model=r['model'], scenario=r['scenario'], run=r['run'], outcome=r['outcome'],
                   pose_samples=len(r['pose']), steering_samples=len(r['trace'])) for r in records]
    ), ensure_ascii=False, indent=2)+'\n')
    print(output/'trajectories.png')
    print(output/'steering.png')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('comparison_directory', type=Path)
    main(parser.parse_args().comparison_directory.resolve())
