#!/usr/bin/env python3
"""Compare recorded trajectories and body yaw for the C5 solver experiment."""
import argparse
import csv
import json
import os
from pathlib import Path

os.environ.setdefault('MPLCONFIGDIR', '/tmp/c5_motion_matplotlib')
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
from std_msgs.msg import String
import yaml

COLORS = {'hybrid': '#D88137', 'bnb': '#1878B5'}
LABELS = {'hybrid': '기존 Hybrid', 'bnb': '전 구간 BnB'}


def extract(root, output):
    plan = json.loads((root / 'plan.json').read_text())
    progress = json.loads((root / 'progress.json').read_text())
    records = []
    for job in plan['jobs']:
        session = Path(progress['completed'][job['id']])
        config = yaml.safe_load((session / 'effective_config.yaml').read_text())
        scalar_yaw = config['lower_tracking_mpc_node']['ros__parameters']['odom_yaw_is_orientation_z']
        summary, = session.glob('results/*/summary.csv')
        with summary.open() as stream:
            attempts = list(csv.DictReader(stream))
        route = np.loadtxt(session / 'routes' / f"{job['scenario']}.csv",
                           delimiter=',', skiprows=1)[:, :2]
        phases, poses = {}, []
        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=str(session / 'bag'), storage_id='mcap'),
                    rosbag2_py.ConverterOptions('cdr', 'cdr'))
        reader.set_filter(rosbag2_py.StorageFilter(topics=[
            '/knu_batch/status', '/motive/vehicle/odom_map']))
        while reader.has_next():
            topic, raw, stamp = reader.read_next()
            if topic == '/knu_batch/status':
                status = json.loads(deserialize_message(raw, String).data)
                phases[status['sequence']] = status
                continue
            msg = deserialize_message(raw, Odometry)
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            q = msg.pose.pose.orientation
            yaw = q.z if scalar_yaw else np.arctan2(2 * (q.w * q.z + q.x * q.y),
                                                   1 - 2 * (q.y * q.y + q.z * q.z))
            poses.append([t if t else stamp * 1e-9,
                          msg.pose.pose.position.x, msg.pose.pose.position.y, yaw])
        del reader
        poses = np.asarray(poses)
        poses = poses[np.argsort(poses[:, 0], kind='stable')]
        phases = sorted(phases.values(), key=lambda s: s['phase_stamp_sec'])
        for number, attempt in enumerate(attempts, 1):
            start = next(s['phase_stamp_sec'] for s in phases
                         if s.get('token') == attempt['token'] and s['phase'] == 'running')
            end = next(s['phase_stamp_sec'] for s in phases
                       if s['phase_stamp_sec'] > start and s['phase'] != 'running')
            pose = poses[(poses[:, 0] >= start) & (poses[:, 0] < end)].copy()
            if len(pose) < 2 or not np.isfinite(pose).all():
                raise ValueError(f"Missing pose: {job['id']} run {number}")
            pose[:, 0] -= start
            pose[:, 3] = np.unwrap(pose[:, 3])
            # Only choose a common 360-degree branch; do not fit heading offsets.
            reference_start = 2.066348 if job['scenario'] == 'scenario1' else -np.pi / 2
            pose[:, 3] += 2 * np.pi * np.round((reference_start - pose[0, 3]) / (2 * np.pi))
            record = dict(mode=job['set'], scenario=job['scenario'], run=number,
                          outcome=attempt['outcome'], pose=pose, route=route, duration=end-start)
            records.append(record)
            name = f"{job['set']}_{job['scenario']}_run{number}"
            np.savez_compressed(output / 'data' / f'{name}.npz', pose_time_xy_yaw=pose, route=route)
            np.savetxt(output / 'data' / f'{name}_pose.csv',
                       np.column_stack((pose[:, :3], np.degrees(pose[:, 3]))), delimiter=',',
                       header='elapsed_sec,map_x_m,map_y_m,body_yaw_deg_unwrapped', comments='')
    if len(records) != 8:
        raise ValueError(f'Expected 8 attempts, found {len(records)}')
    return records


def main(root):
    output = root / 'plots'
    (output / 'data').mkdir(parents=True, exist_ok=True)
    font = Path('/usr/share/fonts/truetype/nanum/NanumGothic.ttf')
    if font.exists():
        plt.rcParams['font.family'] = FontProperties(fname=str(font)).get_name()
    plt.rcParams.update({'axes.unicode_minus': False, 'font.size': 11,
                         'axes.spines.top': False, 'axes.spines.right': False,
                         'savefig.facecolor': 'white', 'pdf.fonttype': 42})
    records = extract(root, output)
    handles = [Line2D([], [], color=COLORS[m], lw=2.5, label=LABELS[m]) for m in COLORS]
    fig, axes = plt.subplots(1, 2, figsize=(12, 7))
    for ax, scenario in zip(axes, ('scenario1', 'scenario3')):
        selected = [r for r in records if r['scenario'] == scenario]
        route = selected[0]['route']
        ax.plot(route[:, 0], route[:, 1], ':o', color='#555', ms=3, lw=1.3, zorder=1)
        for r in selected:
            xy = r['pose'][:, 1:3]
            ax.plot(xy[:, 0], xy[:, 1], color=COLORS[r['mode']],
                    ls='-' if r['run'] == 1 else '--', lw=1.8, alpha=.85, zorder=3)
        ax.scatter(route[0, 0], route[0, 1], marker='s', s=45, facecolor='white',
                   edgecolor='#444', zorder=5)
        ax.annotate('출발', route[0], xytext=(7, -15), textcoords='offset points', fontsize=10)
        ax.scatter(route[-1, 0], route[-1, 1], marker='*', s=95, color='#555', zorder=5)
        ax.set(title=f"시나리오 {scenario[-1]}", xlabel='지도 X (m)', ylabel='지도 Y (m)')
        ax.set_aspect('equal', adjustable='datalim')
        ax.grid(alpha=.2)
        ax.margins(.13)
    style_handles = [Line2D([], [], color='#555', ls='-', label='1회차'),
                     Line2D([], [], color='#555', ls='--', label='2회차'),
                     Line2D([], [], color='#555', ls=':', marker='o', ms=3, label='기준 웨이포인트')]
    fig.legend(handles=handles + style_handles, loc='upper center',
               bbox_to_anchor=(.5, .94), ncol=3, frameon=False)
    fig.suptitle('솔버 변경 전후 · 주행 자취', fontsize=18, y=.995)
    fig.text(.5, .025, '각 시나리오·방식별 2회 / 총 8회 모두 완주 · 지도에 정렬한 EKF 위치 · 주행 구간',
             ha='center', color='#555', fontsize=10)
    fig.subplots_adjust(top=.79, bottom=.13, left=.085, right=.97, wspace=.29)
    for ext in ('png', 'pdf'):
        fig.savefig(output / f'trajectories.{ext}', dpi=190)
    plt.close(fig)

    fig, axes = plt.subplots(2, 2, figsize=(12, 7.6))
    for row, scenario in enumerate(('scenario1', 'scenario3')):
        selected = [r for r in records if r['scenario'] == scenario]
        lo = min(np.degrees(r['pose'][:, 3]).min() for r in selected)
        hi = max(np.degrees(r['pose'][:, 3]).max() for r in selected)
        margin = max(4, .06 * (hi - lo))
        max_time = max(r['duration'] for r in selected)
        for col, run in enumerate((1, 2)):
            ax = axes[row, col]
            for r in selected:
                if r['run'] == run:
                    ax.plot(r['pose'][:, 0], np.degrees(r['pose'][:, 3]),
                            color=COLORS[r['mode']], lw=1.6, alpha=.9)
            ax.set(title=f'시나리오 {scenario[-1]} · {run}회차',
                   xlabel='출발 후 시간 (초)', ylabel='차량 헤딩 (도)',
                   xlim=(0, max_time + .5), ylim=(lo-margin, hi+margin))
            ax.grid(alpha=.2)
    fig.legend(handles=handles, loc='upper center', bbox_to_anchor=(.5, .94), ncol=2, frameon=False)
    fig.suptitle('솔버 변경 전후 · 차량 헤딩', fontsize=18, y=.995)
    fig.text(.5, .02, '동쪽 0° · 반시계방향 양수 (지도 ENU) · EKF 차량 방향 · 회차별 출발 시각만 0초로 정렬',
             ha='center', color='#555', fontsize=10)
    fig.subplots_adjust(top=.84, bottom=.11, left=.085, right=.98, hspace=.48, wspace=.23)
    for ext in ('png', 'pdf'):
        fig.savefig(output / f'heading.{ext}', dpi=190)
    plt.close(fig)
    (output / 'motion_manifest.json').write_text(json.dumps(dict(
        source='/motive/vehicle/odom_map', pose='EKF, map-aligned',
        heading='body yaw decoded using each frozen config, ENU, continuous branch in degrees',
        alignment='running-phase start time; only whole 360-degree yaw branch alignment',
        runs=[dict(mode=r['mode'], scenario=r['scenario'], run=r['run'], outcome=r['outcome'],
                   samples=len(r['pose']), duration_sec=r['duration']) for r in records]
    ), ensure_ascii=False, indent=2) + '\n')
    print(output / 'trajectories.png')
    print(output / 'heading.png')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('comparison_directory', type=Path)
    main(parser.parse_args().comparison_directory.resolve())
