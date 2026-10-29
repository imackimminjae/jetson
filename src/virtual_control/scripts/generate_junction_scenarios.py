#!/usr/bin/env python3
"""Create short, position-gated bias missions from completed zero-bias drives.

Recorded poses supply road references and spawn headings, not a new tracking
target. Missions retain the parent SDMap points, with sparse window gates added.
The plotted reference and predicted window are design evidence, not new runs.
"""
import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path

os.environ.setdefault('MPLCONFIGDIR', '/tmp/junction_scenarios_matplotlib')
os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')
import numpy as np
from scipy.ndimage import distance_transform_edt, map_coordinates

WORKSPACE = Path(__file__).resolve().parents[3]
PACKAGE = Path(__file__).resolve().parents[1]
SOURCE = WORKSPACE / ('parameter_sweeps/'
    'manual_bias_scenario22_scenario23_scenario24_scenario25_20261030_052032_63228b/plots/data')
OUTPUT = WORKSPACE / 'diagnostics/local_bias_scenarios_20261030'
DESIGNS = {
    26: dict(parent=22, area='west_roundabout', center=[-120., 40.],
             label='서측 원형 교차로 · 서쪽 → 동쪽', before=75., after=75., on=-35., off=25.),
    27: dict(parent=25, area='west_roundabout', center=[-120., 40.],
             label='서측 원형 교차로 · 남쪽 → 동쪽', before=75., after=75., on=-35., off=25.),
    28: dict(parent=23, area='southeast_forks', center=[105., -15.],
             label='남동 분기 · 서쪽 → 동쪽', before=75., after=75., on=-35., off=15.),
    29: dict(parent=24, area='southeast_forks', center=[105., -15.],
             label='남동 분기 · 남쪽 → 북동쪽', before=75., after=75., on=-35., off=25.),
    30: dict(parent=22, area='northeast_junction', center=[195., 140.],
             label='북동 교차로 · 남서쪽 → 동쪽', before=75., after=65., on=-35., off=20.),
    31: dict(parent=25, area='northeast_junction', center=[195., 140.],
             label='북동 교차로 · 남서쪽 → 북쪽', before=75., after=60., on=-35., off=20.),
}


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def save_json(path, value):
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2,
                                    allow_nan=False) + '\n')


def arc(xy):
    return np.r_[0., np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1))]


def observed_margin_function():
    filename = WORKSPACE / 'diagnostics/isaac_observed_roads_20261025/shared_isaac_road_mosaic.npz'
    mosaic = np.load(filename)
    road, resolution = mosaic['road'], float(mosaic['resolution_m'])
    signed = (distance_transform_edt(road) - distance_transform_edt(~road)) * resolution
    low = mosaic['map_low']

    def margin(xy):
        pixels = (np.asarray(xy) - low) / resolution - .5
        return map_coordinates(signed, [pixels[:, 1], pixels[:, 0]],
                               order=1, mode='constant', cval=-1000.)

    return margin, filename


def generate(source=SOURCE):
    margin, mosaic_path = observed_margin_function()
    result = {}
    for number, design in DESIGNS.items():
        filename = Path(source) / f'scenario{design["parent"]}_run1.npz'
        data = np.load(filename)
        pose = data['pose_time_xy_yaw']
        xy, station = pose[:, 1:3], arc(pose[:, 1:3])
        # Remove stationary duplicate positions before interpolation.
        keep = np.r_[True, np.diff(station) > 1e-8]
        pose, xy, station = pose[keep], xy[keep], station[keep]
        center_index = int(np.linalg.norm(xy - design['center'], axis=1).argmin())
        center = float(station[center_index])
        start, end = center - design['before'], min(center + design['after'], station[-1])
        # findWaypointPair advances when the current wp0 is within 10 m ahead.
        # Place gates 10 m beyond the desired on/off positions; replay below
        # reports the actual predicted switch, including curved approaches.
        on_gate = center + design['on'] + 10.
        off_gate = center + design['off'] + 10.
        if not 0. < start < on_gate < off_gate < end - 15.:
            raise ValueError(f'S{number}: insufficient approach or recovery')

        def point(s):
            return np.array([np.interp(s, station, xy[:, k]) for k in (0, 1)])

        # A point after the off gate is essential: the final waypoint pair
        # cannot advance. Keep a separate recovery point and terminal goal.
        controls = [(start, 'start'), (on_gate, 'bias_on_gate'),
                    (off_gate, 'bias_off_gate'),
                    ((off_gate + end) / 2., 'recovery'), (end, 'goal')]
        parent_route = data['route_xy']
        parent_stations = []
        for i, waypoint in enumerate(parent_route):
            s = float(station[np.linalg.norm(xy - waypoint, axis=1).argmin()])
            if start + 8. < s < end - 8. and all(abs(s - value) >= 8. for value, _ in controls):
                controls.append((s, f'parent_wp_{i + 1}'))
                parent_stations.append(i + 1)
        controls.sort()
        points = np.round(np.array([
            parent_route[int(kind.removeprefix('parent_wp_')) - 1]
            if kind.startswith('parent_wp_') else point(s)
            for s, kind in controls]), 6)
        kinds = [kind for _, kind in controls]
        # Window numbers are 1-based current wp0 values, after each gate.
        window = [kinds.index('bias_on_gate') + 2, kinds.index('bias_off_gate') + 1]
        if not 3 <= window[0] <= window[1] < len(points) - 1:
            raise ValueError(f'S{number}: window cannot deactivate before the last pair')
        sample_stations = np.arange(start, end, .25)
        sample_stations = np.r_[sample_stations, end]
        reference = np.array([point(s) for s in sample_stations])
        heading = float(np.interp(start, station, pose[:, 3]))
        direction = np.array([math.cos(heading), math.sin(heading)])
        normal = np.array([-direction[1], direction[0]])
        footprint = np.array([points[0] + direction * l + normal * w
                              for l in [-2.35, 0., 3.8] for w in [-.9, 0., .9]])
        spawn_margin = float(margin(footprint).min())
        if spawn_margin <= .25:
            raise ValueError(f'S{number}: spawn body clearance {spawn_margin:.3f} m')
        clipped = pose[(station >= start) & (station <= end)].copy()
        clipped[:, 0] -= clipped[0, 0]
        metadata = dict(
            scenario=f'scenario{number}', label=design['label'], area=design['area'],
            parent_scenario=f'scenario{design["parent"]}', parent_run=1,
            coordinate_frame='map', source_recording=str(filename.relative_to(WORKSPACE)),
            source_recording_sha256=sha256(filename),
            source_station_bounds_m=[float(start), float(end)],
            center_hint_xy_m=design['center'],
            start=points[0].tolist(), end=points[-1].tolist(), heading_rad=heading,
            length_m=float(arc(points)[-1]), reference_length_m=float(end - start),
            nominal_seconds_at_6mps=float((end - start) / 6.),
            waypoint_count=len(points), waypoint_kinds=kinds,
            preserved_parent_waypoint_numbers=parent_stations,
            bias_mode='waypoint_window', bias_waypoints=window,
            requested_bias_station_bounds_m=[float(center + design['on'] - start),
                                             float(center + design['off'] - start)],
            default_bias_degrees=[0, -5, 5],
            spawn_observed_road_body_margin_m=spawn_margin,
            observed_road_source=str(mosaic_path.relative_to(WORKSPACE)),
            observed_road_source_sha256=sha256(mosaic_path),
            route_progress_and_goal='original short mission, independent of bias',
            bias_anchor='measured vehicle position; planning waypoints only',
            actual_drive_validated=False,
            validation='parent completed at zero bias; short mission not yet driven',
        )
        result[number] = dict(points=points, reference=reference,
                              reference_station=sample_stations - start,
                              pose=clipped, metadata=metadata)
    return result


def write_routes(result):
    directory = PACKAGE / 'data/knu_routes'
    for number, item in result.items():
        with (directory / f'scenario{number}.csv').open('w', newline='') as stream:
            writer = csv.writer(stream)
            writer.writerow(['x', 'y', 'z'])
            writer.writerows([*point, 0.] for point in item['points'])
    starts_path = directory / 'knu_start_poses.json'
    starts = json.loads(starts_path.read_text())
    names = {f'scenario{n}' for n in result}
    starts['starts'] = [row for row in starts['starts'] if row['scenario'] not in names]
    header = ['#pragma once', '#include "virtual_control/px4_odom_map_transform.hpp"',
              'namespace virtual_control { namespace px4_odom_map {',
              'inline std::optional<RouteAnchor> junctionRouteAnchorForName(const std::string & route_name)', '{']
    for number, item in result.items():
        meta = item['metadata']
        first = [*meta['start'], 0.]
        yaw = meta['heading_rad']
        second = [first[0] + 5. * math.cos(yaw), first[1] + 5. * math.sin(yaw), 0.]
        starts['starts'].append(dict(scenario=meta['scenario'], position=first,
            path_first=first, heading_rad=yaw, ros_heading_rad=yaw,
            profile='local_bias6', reference_mask_body_on_road=True,
            reference_mask_clearance_m=meta['spawn_observed_road_body_margin_m'],
            validation=meta['validation']))
        a, b = (', '.join(f'{v:.12f}' for v in values) for values in [first, second])
        header.extend([f'  if (route_name == "scenario{number}") {{',
                       f'    return RouteAnchor{{{{{a}}}, {{{b}}}}};', '  }'])
    header.extend(['  return std::nullopt;', '}', '} }  // namespace virtual_control::px4_odom_map', ''])
    (PACKAGE / 'include/virtual_control/knu_junction_route_anchors.hpp').write_text('\n'.join(header))
    starts['version'] = max(6, starts['version'])
    starts['additional_profile'] = 'long4: scenarios22-25; local_bias6: scenarios26-31; original starts1-21 retained'
    save_json(starts_path, starts)
    profile_path = directory / 'knu_batch_profile.json'
    profile = json.loads(profile_path.read_text())
    profile['additional_routes'] = list(dict.fromkeys(profile.get('additional_routes', []) + list(sorted(names))))
    profile.setdefault('additional_profiles', {})['local_bias6'] = 'knu_junction_profile.json'
    save_json(profile_path, profile)
    save_json(directory / 'knu_junction_profile.json', dict(
        version=1, profile='local_bias6', coordinate_frame='map',
        routes=[f'scenario{n}' for n in result], route_count=len(result),
        default_batch_unchanged=True, actual_drive_validated=False,
        confirmed_controller=dict(lower_state_count=3, steering_tau_sec=.74, force_bnb=True),
        policy='Sparse guidance from completed zero-bias parents. Window gates advance '
               'using nominal mission progress, not elapsed time. Separate off gate, '
               'recovery waypoint and goal ensure deactivation before the final pair. '
               'No short-mission driving results are claimed.',
        definitions=[item['metadata'] for item in result.values()]))


def save_design(result, destination):
    destination.mkdir(parents=True, exist_ok=True)
    (destination / 'references').mkdir(exist_ok=True)
    (destination / 'gating_inputs').mkdir(exist_ok=True)
    for number, item in result.items():
        np.savez_compressed(destination / 'references' / f'scenario{number}.npz',
                            xy=item['reference'], station_m=item['reference_station'],
                            source_pose_time_xy_yaw=item['pose'])
        # Inputs for independent replay of the actual C++ waypoint-pair code.
        np.savetxt(destination / 'gating_inputs' / f'scenario{number}_poses.csv',
                   item['pose'], delimiter=',', header='t,x,y,yaw', comments='')
    save_json(destination / 'design_report.json', dict(
        status='designed_not_driven', definitions=[i['metadata'] for i in result.values()]))


def render(result, destination, gate_report=None):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.font_manager import FontProperties
    from matplotlib.lines import Line2D
    plt.rcParams.update({'font.family': FontProperties(
        fname='/usr/share/fonts/truetype/nanum/NanumGothic.ttf').get_name(),
        'axes.unicode_minus': False, 'font.size': 10, 'pdf.fonttype': 42})
    mosaic = np.load(WORKSPACE / 'diagnostics/isaac_observed_roads_20261025/shared_isaac_road_mosaic.npz')
    road = mosaic['road']
    background = np.where(road, .91, 1.)
    low, high = mosaic['map_low'], mosaic['map_high']
    fig, axes = plt.subplots(2, 3, figsize=(16., 10.6))
    for number, ax in zip([26, 28, 30, 27, 29, 31], axes.flat):
        item, meta = result[number], result[number]['metadata']
        xy, station = item['reference'], item['reference_station']
        on, off = meta['requested_bias_station_bounds_m']
        if gate_report:
            on, off = gate_report[f'scenario{number}']['reference_bias_station_bounds_m']
        start_i, end_i = np.searchsorted(station, [on, off])
        ax.imshow(background, origin='lower', extent=[low[0], high[0], low[1], high[1]],
                  cmap='gray', vmin=0., vmax=1., interpolation='nearest', zorder=0)
        for segment, color in [(xy[:start_i + 1], '#2576b7'),
                               (xy[start_i:end_i + 1], '#e47d25'),
                               (xy[end_i:], '#268768')]:
            ax.plot(segment[:, 0], segment[:, 1], lw=3.2, color=color, zorder=3)
            if len(segment) > 6:
                i = len(segment) // 2
                a, b = max(0, i - 8), min(len(segment) - 1, i + 8)
                ax.annotate('', xy=segment[b], xytext=segment[a],
                            arrowprops=dict(arrowstyle='->', lw=1.5, color=color), zorder=6)
        p = item['points']
        ax.scatter(p[:, 0], p[:, 1], s=24, facecolor='white', edgecolor='#555', zorder=5)
        ax.scatter(*p[0], s=60, marker='s', facecolor='white', edgecolor='#2576b7', linewidth=1.8, zorder=7)
        ax.scatter(*p[-1], s=110, marker='*', color='#268768', zorder=7)
        for s, text, color in [(on, 'ON', '#b05210'), (off, 'OFF', '#16664d')]:
            pos = xy[min(len(xy) - 1, np.searchsorted(station, s))]
            ax.annotate(text, pos, xytext=(8, 10), textcoords='offset points',
                        fontsize=10, fontweight='bold', color=color,
                        bbox=dict(fc='white', ec='none', alpha=.85, pad=1.5), zorder=8)
        lengths = [on, off - on, meta['reference_length_m'] - off]
        ax.set_title(f'S{number} · {meta["label"]}\n'
                     f'약 {meta["reference_length_m"]:.0f} m  |  진입 {lengths[0]:.0f} · 바이어스 {lengths[1]:.0f} · 회복 {lengths[2]:.0f} m',
                     fontsize=11, pad=9)
        lo, hi = xy.min(axis=0), xy.max(axis=0)
        center = (lo + hi) / 2.
        half = max(hi - lo) / 2. + 16.
        ax.set(xlim=(center[0] - half, center[0] + half), ylim=(center[1] - half, center[1] + half),
               xlabel='지도 X (m)', ylabel='지도 Y (m)')
        ax.set_aspect('equal'); ax.grid(alpha=.14)
    fig.suptitle('부분 방향 바이어스용 짧은 시나리오 6개 · 설계안', fontsize=19, y=.99)
    handles = [Line2D([], [], color=c, lw=3, label=l) for c, l in [
        ('#2576b7', '정상 진입'), ('#e47d25', '바이어스 적용 구간'), ('#268768', '해제 후 회복')]]
    handles += [Line2D([], [], color='#555', marker='o', mfc='white', ls='', label='SDMap 웨이포인트'),
                Line2D([], [], color='#2576b7', marker='s', mfc='white', ls='', label='출발'),
                Line2D([], [], color='#268768', marker='*', ms=10, ls='', label='종료 목표')]
    fig.legend(handles=handles, loc='upper center', bbox_to_anchor=(.5, .953), ncol=6, frameon=False)
    fig.text(.5, .018, '3-state (τ=0.74초) + BnB 유지 · 비교 조건 0°, -5°, +5° · 새 시나리오는 아직 주행하지 않음\n'
             '실선: 기존 완주 기록을 잘라 만든 도로 참고선 · ON/OFF: 기존 기록에 원래 웨이포인트 전환 규칙을 적용한 예상 위치',
             ha='center', fontsize=10, color='#555', linespacing=1.6)
    fig.subplots_adjust(top=.87, bottom=.11, left=.06, right=.985, wspace=.26, hspace=.35)
    for extension in ['png', 'pdf']:
        fig.savefig(destination / f'short_routes.{extension}', dpi=170, facecolor='white')
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--write', action='store_true', help='Register S26–S31 without changing the default batch')
    parser.add_argument('--gate-report', type=Path, help='Actual C++ waypoint-pair replay report for rendering')
    parser.add_argument('--no-plot', action='store_true', help='Write data only before waypoint-gate replay')
    args = parser.parse_args()
    result = generate()
    report = json.loads(args.gate_report.read_text()) if args.gate_report else None
    if report:
        for number, item in result.items():
            replay = report[f'scenario{number}']
            if not np.array_equal(replay['mission_waypoints_xy'], item['points']):
                raise ValueError(f'S{number}: gate replay belongs to different waypoints')
            item['metadata']['reference_replay'] = replay
    if args.write:
        write_routes(result)
    save_design(result, OUTPUT)
    if not args.no_plot:
        render(result, OUTPUT, report)
    for item in result.values():
        meta = item['metadata']
        print(meta['scenario'], round(meta['reference_length_m'], 1), 'm',
              'window', meta['bias_waypoints'], 'spawn clearance', round(meta['spawn_observed_road_body_margin_m'], 2))


if __name__ == '__main__':
    main()
