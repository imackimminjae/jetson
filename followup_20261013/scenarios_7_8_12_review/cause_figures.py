"""Selected latest local BEV frames; no accumulated maps or full-grid cache."""
from pathlib import Path
import csv
import gzip
import json
import math
import os

os.environ['ROS_DOMAIN_ID'] = '177'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
os.environ['ROS_LOG_DIR'] = '/tmp/s7812_cause_roslog'
os.environ['MPLCONFIGDIR'] = '/tmp/s7812_cause_mpl'
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import OccupancyGrid
import matplotlib
matplotlib.use('Agg')
from matplotlib import pyplot as plt, font_manager
from matplotlib.colors import ListedColormap, BoundaryNorm
from matplotlib.lines import Line2D
from matplotlib.backends.backend_pdf import PdfPages

ROOT = Path('/home/imac/ros2_ws')
OUT = Path(__file__).resolve().parent
BATCHES = {7: '20261013_013718_scenario7_74622c', 8: '20261013_014149_scenario8_b5ef30',
           12: '20261013_015055_scenario12_0c1227'}
COLORS = ['#0072B2', '#D55E00', '#009E73', '#CC79A7', '#E69F00']
font_manager.fontManager.addfont('/usr/share/fonts/truetype/nanum/NanumGothic.ttf')
plt.rcParams.update({'font.family': 'NanumGothic', 'axes.unicode_minus': False})
cases = {7: [(3, (-115, 61))], 8: [(4, (-120, 40)), (2, (-120, 40))],
         12: [(3, (253, 140)), (2, (253, 140)), (5, (253, 136))]}
limits = {7: ((-134, -76), (36, 96)), 8: ((-143, -91), (20, 75)),
          12: ((239, 277), (127, 164))}
records, figs = [], []
for n, selections in cases.items():
    folder = OUT/f'scenario{n}'
    events = json.load(gzip.open(folder/'events.json.gz', 'rt'))
    ats = list(csv.DictReader((folder/'per_attempt.csv').open()))
    data = np.load(folder/'signals.npz')
    bs, od = data['B'], data['odom']
    upper = json.load(gzip.open(folder/'upper_trace.json.gz', 'rt'))
    route = np.array([[float(r['x']), float(r['y'])] for r in csv.DictReader((ROOT/'drive_batches'/BATCHES[n]/f'routes/scenario{n}.csv').open())])
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(ROOT/'drive_batches'/BATCHES[n]/'bag'), storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    reader.set_filter(rosbag2_py.StorageFilter(topics=['/bev/occupancy_grid']))
    fig, axes = plt.subplots(1, len(selections), figsize=(6*len(selections), 7), squeeze=False)
    for ax, (i, target) in zip(axes.flat, selections):
        a = ats[i-1]
        start, end = float(a['start_epoch_s']), float(a['outcome_epoch_s'])
        ev = [e for e in events if start <= e['t_start'] < end]
        e = min(ev, key=lambda e: (e['x']-target[0])**2+(e['y']-target[1])**2)
        state = bs[np.argmin(abs(bs[:, 0]-e['t_emit']))]
        trace = min(upper, key=lambda v: abs(v[0]-e['t_emit']))
        assert abs(trace[0]-e['t_emit']) < .1
        reader.seek(int((e['t_start']-1)*1e9))
        last = None
        while reader.has_next():
            _, blob, t = reader.read_next()
            if t > int(e['t_start']*1e9):
                break
            last = blob, t
        assert last is not None
        blob, stamp = last
        grid = deserialize_message(blob, OccupancyGrid)
        raster = np.asarray(grid.data, dtype=np.int8).reshape(grid.info.height, grid.info.width)
        q = grid.info.origin.orientation
        grid_yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        gx, gy = np.meshgrid(np.arange(grid.info.width+1)*grid.info.resolution,
                             np.arange(grid.info.height+1)*grid.info.resolution)
        c, s = math.cos(grid_yaw), math.sin(grid_yaw)
        bx = grid.info.origin.position.x+c*gx-s*gy
        by = grid.info.origin.position.y+s*gx+c*gy
        c, s = math.cos(e['yaw_rad']), math.sin(e['yaw_rad'])
        wx, wy = e['x']+c*bx-s*by, e['y']+s*bx+c*by
        categories = np.where(raster < 0, 2, np.where(raster <= 1, 0, 1))
        ax.pcolormesh(wx, wy, categories, cmap=ListedColormap(['white', '#575757', '#e8cf8b']),
                      norm=BoundaryNorm([-.5,.5,1.5,2.5],3), shading='flat', rasterized=True)
        for w in e['intervals_world']:
            _, _, x0, y0, x1, y1 = w[:6]
            ax.plot([x0,x1], [y0,y1], color='#2c9d69', lw=1.6)
        preview = np.array(e['preview_world'])
        if len(preview):
            ax.plot(preview[:, 0], preview[:, 1], '--', color='#e17b1b', lw=1.5)
        plan = np.array(e['published_world'] or e.get('retained_world', []))
        if len(plan):
            ax.plot(plan[:, 0], plan[:, 1], '-' if e['valid_plan'] else '--', color='#d51a46', lw=2)
        tr = od[(od[:, 0] >= start) & (od[:, 0] < end)]
        ax.plot(tr[:, 1], tr[:, 2], color=COLORS[i-1], lw=1.2, alpha=.6)
        ax.plot(route[:, 0], route[:, 1], 'k:o', ms=3, lw=1.2)
        ax.plot(e['x'], e['y'], 'k^', ms=8)
        # Desired road direction from the logged incoming/outgoing headings, not guessed from the picture.
        road_heading = trace[12]+trace[3]
        ax.annotate('', xy=(e['x']+9*math.cos(road_heading), e['y']+9*math.sin(road_heading)),
                    xytext=(e['x'], e['y']), arrowprops=dict(arrowstyle='->', color='#6923af', lw=2))
        ax.set_xlim(*limits[n][0])
        ax.set_ylim(*limits[n][1])
        ax.set_aspect('equal')
        ax.set_facecolor('#e6eaf0')
        ax.grid(alpha=.13)
        ax.set_xlabel('map X [m]')
        ax.set_ylabel('map Y [m]')
        ax.set_title(f'{i}회 · '+('목표 접근' if a['outcome']=='completed' else 'off_route')+
                     f'\nB {state[2]:.1f}m · 예측 {e["horizon"]}단계 · '+('계획 발행' if e['valid_plan'] else '계획 실패'), fontsize=11)
        np.savez_compressed(folder/f'case_a{i}_grid.npz', data=raster)
        records.append(dict(scenario=n, attempt=i, event=e,
                            B_after_m=float(state[2]), grid_receive_epoch_s=stamp/1e9,
                            grid_before_plan_ms=1000*(e['t_start']-stamp/1e9),
                            desired_road_heading_deg=math.degrees(road_heading),
                            previous_reference_used=bool(trace[-1]),
                            upper_trace=trace))
    fig.suptitle(f'시나리오{n} · 분기 구간의 계획·참조·인터벌 비교', fontsize=15, y=.98)
    handles = [Line2D([], [], color='#d51a46', lw=2, label='발행 계획 / 점선은 유지한 계획'),
               Line2D([], [], color='#e17b1b', ls='--', label='최적화 전 참조'),
               Line2D([], [], color='#2c9d69', label='처리 후 인터벌'),
               Line2D([], [], color='#6923af', lw=2, label='로그의 요구 출구 방향')]
    fig.legend(handles=handles, loc='lower center', bbox_to_anchor=(.5,.095), ncol=2 if n==7 else 4, fontsize=8)
    fig.text(.04,.023,'배경은 각 계획 시작 직전의 단일 BEV. 헤더 시각0·사용 grid ID 미기록으로 최신 선행 수신 기록에 대응.\n'
             '계획 pose로 map 좌표에 표시. 옅은 청회색=BEV 밖. 흐린 회차색은 구간 전체 실제 궤적이며 배경과 동시각이 아님.', fontsize=8)
    fig.subplots_adjust(top=.85, bottom=.23, wspace=.13)
    fig.savefig(OUT/f'scenario{n}_branch_bev.png', dpi=170)
    fig.savefig(OUT/f'scenario{n}_branch_bev.pdf')
    figs.append(fig)
with PdfPages(OUT/'BRANCH_BEV_COMPARISON.pdf') as pdf:
    for fig in figs:
        pdf.savefig(fig)
plt.close('all')
(OUT/'selected_branch_cycles.json').write_text(json.dumps(records, ensure_ascii=False, indent=2)+'\n')
print('분기 비교 BEV 6장, 그림3개 저장 완료')
