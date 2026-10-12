"""Two suspect regions, one selected BEV per region and attempt; no accumulation."""
from pathlib import Path
from collections import Counter
from datetime import datetime
from zoneinfo import ZoneInfo
import csv
import gzip
import hashlib
import json
import math
import os

os.environ['ROS_DOMAIN_ID'] = '177'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
os.environ['ROS_LOG_DIR'] = '/tmp/s1_bev_zoom_roslog'
os.environ['MPLCONFIGDIR'] = '/tmp/s1_bev_zoom_mpl'
import numpy as np
import yaml
import rosbag2_py
from rclpy.serialization import deserialize_message
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
import matplotlib
matplotlib.use('Agg')
from matplotlib import pyplot as plt, font_manager
from matplotlib.colors import ListedColormap, BoundaryNorm
from matplotlib.lines import Line2D
from matplotlib.patches import Patch
from matplotlib.backends.backend_pdf import PdfPages

ROOT = Path('/home/imac/ros2_ws')
OUT = Path(__file__).resolve().parent
BATCH = ROOT / 'drive_batches/20261012_215236_scenario1_9e41f2'
attempts = json.loads((OUT.parent / 'scenario1_attempts.json').read_text())
odom = np.array([[float(r[k]) for k in ['received_epoch_s', 'x_map_m', 'y_map_m']]
                 for r in csv.DictReader(gzip.open(OUT.parent / 'scenario1_odom.csv.gz', 'rt'))])
route = np.array([[float(r['x']), float(r['y'])] for r in csv.DictReader((BATCH / 'routes/scenario1.csv').open())])
config = yaml.safe_load((BATCH / 'effective_config.yaml').read_text())['upper_planner_node']['ros__parameters']
assert not config['grid_origin_x_override_enabled']
assert not config['enable_waypoint_bias']
assert config['grid_value_threshold'] == 1 and not config['grid_positive_is_drivable']
reader = rosbag2_py.SequentialReader()
reader.open(rosbag2_py.StorageOptions(uri=str(BATCH / 'bag'), storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
reader.set_filter(rosbag2_py.StorageFilter(topics=['/debug/upper_branch_event']))
events = []
while reader.has_next():
    _, blob, _ = reader.read_next()
    event = json.loads(deserialize_message(blob, String).data)
    if -70 < event['y'] < -20:
        events.append(event)

font_manager.fontManager.addfont('/usr/share/fonts/truetype/nanum/NanumGothic.ttf')
plt.rcParams.update({'font.family': 'NanumGothic', 'axes.unicode_minus': False, 'font.size': 9})
colors = ['#0072B2', '#D55E00', '#009E73', '#CC79A7', '#E69F00']
regions = [dict(id=1, name='첫 번째 굴곡', target_y=-62, xlim=(94, 125), ylim=(-66, -24)),
           dict(id=2, name='두 번째 회전', target_y=-30, xlim=(99, 147), ylim=(-35, 5))]
reader.set_filter(rosbag2_py.StorageFilter(topics=['/bev/occupancy_grid']))
selected, geometry = [], []
figures = []
legend = [Patch(facecolor='white', edgecolor='#999', label='BEV 주행 가능'),
          Patch(facecolor='#575757', label='BEV 비주행 영역'),
          Patch(facecolor='#e6eaf0', label='BEV 범위 밖'),
          Line2D([], [], color='black', ls='--', label='원래 waypoint'),
          Line2D([], [], color='#3975a3', lw=2, label='해당 회차 궤적'),
          Line2D([], [], color='black', marker='^', ls='none', label='BEV 대응 차량 위치')]
for region in regions:
    fig, axes = plt.subplots(2, 3, figsize=(14, 10))
    overview = axes.flat[0]
    overview.plot(route[:, 0], route[:, 1], 'k--o', ms=3, lw=1.3)
    for a in attempts:
        tr = odom[(odom[:, 0] >= a['start_epoch_s']) & (odom[:, 0] < a['outcome_epoch_s'])]
        overview.plot(tr[:, 1], tr[:, 2], color=colors[a['attempt']-1], lw=1.5)
    overview.set_title('원래 그림의 해당 구간 · 5회 궤적', fontsize=11)
    for ax, a in zip(list(axes.flat)[1:], attempts):
        es = [e for e in events if a['start_epoch_s'] <= e['t_start'] < a['outcome_epoch_s']]
        e = min(es, key=lambda e: abs(e['y'] - region['target_y']))
        reader.seek(int((e['t_start'] - 1) * 1e9))
        last = None
        while reader.has_next():
            _, blob, stamp = reader.read_next()
            if stamp > int(e['t_start'] * 1e9):
                break
            last = blob, stamp
        assert last is not None
        blob, stamp = last
        grid = deserialize_message(blob, OccupancyGrid)
        data = np.array(grid.data, dtype=np.int8).reshape(grid.info.height, grid.info.width)
        assert data.nbytes == 128 * 128
        q = grid.info.origin.orientation
        origin_yaw = math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        x, y = np.meshgrid(np.arange(grid.info.width+1) * grid.info.resolution,
                           np.arange(grid.info.height+1) * grid.info.resolution)
        c, s = math.cos(origin_yaw), math.sin(origin_yaw)
        bx = grid.info.origin.position.x + c*x - s*y
        by = grid.info.origin.position.y + s*x + c*y
        c, s = math.cos(e['yaw_rad']), math.sin(e['yaw_rad'])
        wx = e['x'] + c*bx - s*by
        wy = e['y'] + s*bx + c*by
        category = np.where(data < 0, 2, np.where(data <= 1, 0, 1))
        ax.pcolormesh(wx, wy, category, shading='flat', rasterized=True,
                      cmap=ListedColormap(['white', '#575757', '#e8cf8b']),
                      norm=BoundaryNorm([-.5, .5, 1.5, 2.5], 3))
        tr = odom[(odom[:, 0] >= a['start_epoch_s']) & (odom[:, 0] < a['outcome_epoch_s'])]
        ax.plot(route[:, 0], route[:, 1], 'k--o', ms=3, lw=1.2)
        ax.plot(tr[:, 1], tr[:, 2], color=colors[a['attempt']-1], lw=1.8)
        # Mark the recorded pose, with a short arrow indicating its scalar yaw.
        ax.plot(e['x'], e['y'], 'k^', ms=7)
        ax.annotate('', xy=(e['x']+2*c, e['y']+2*s), xytext=(e['x'], e['y']),
                    arrowprops=dict(arrowstyle='->', color='black', lw=1.3))
        elapsed = e['t_start'] - a['start_epoch_s']
        age = 1000 * (e['t_start'] - stamp/1e9)
        ax.set_title(f'{a["attempt"]}회 · running +{elapsed:.2f}초\nBEV 기록: 계획 시작 {age:.1f}ms 전', fontsize=10)
        name = f'region{region["id"]}_a{a["attempt"]}'
        np.savez_compressed(OUT / f'{name}_grid.npz', data=data)
        selected.append(dict(region=region['id'], attempt=a['attempt'], event=e,
                             grid_receive_epoch_s=stamp/1e9, grid_age_before_plan_ms=age,
                             grid_header_stamp_sec=grid.header.stamp.sec+grid.header.stamp.nanosec/1e9,
                             time_kst=datetime.fromtimestamp(e['t_start'], ZoneInfo('Asia/Seoul')).isoformat(timespec='milliseconds'),
                             frame_id=grid.header.frame_id, resolution=grid.info.resolution,
                             origin_x=grid.info.origin.position.x, origin_y=grid.info.origin.position.y,
                             origin_yaw_rad=origin_yaw, value_counts={str(k): int(v) for k, v in zip(*np.unique(data, return_counts=True))}))
    for ax in axes.flat:
        ax.set_xlim(*region['xlim'])
        ax.set_ylim(*region['ylim'])
        ax.set_aspect('equal')
        ax.set_facecolor('#e6eaf0' if ax is not overview else 'white')
        ax.grid(alpha=.15)
        ax.set_xlabel('map X [m]')
        ax.set_ylabel('map Y [m]')
    fig.suptitle(f'시나리오1 · {region["name"]} 확대 · 회차별 단일 BEV\n2026-10-12 21:52 배치 · 각 그림은 해당 회차의 실제 궤적', fontsize=15, y=.98)
    fig.legend(handles=legend, loc='lower center', bbox_to_anchor=(.5, .085), ncol=3, fontsize=9)
    fig.text(.04, .024,
             'BEV는 각 계획 시작 직전 최신 기록 한 장이며 해당 계획의 차량 위치·yaw로 표시했습니다. 지도 누적 없음.\n'
             '색 궤적은 해당 구간을 통과한 기록 전체로, 배경 BEV 시각과 모든 궤적 표본 시각이 같지는 않습니다.\n'
             'BEV 헤더 시각=0, 사용 메시지 ID 미기록: 셀과 궤적의 겹침만으로 실제 도로 침범·획득 지연을 확정할 수 없습니다.', fontsize=8)
    fig.subplots_adjust(top=.87, bottom=.21, hspace=.35, wspace=.24)
    fig.savefig(OUT / f'region{region["id"]}_bev_all5.png', dpi=175)
    fig.savefig(OUT / f'region{region["id"]}_bev_all5.pdf')
    figures.append(fig)
    print(f'구간{region["id"]} 5회 단일 BEV 그림 저장', flush=True)
with PdfPages(OUT / 'S1_TWO_REGIONS_BEV.pdf') as pdf:
    for fig in figures:
        pdf.savefig(fig)
plt.close('all')
(OUT / 'selected_frames.json').write_text(json.dumps(selected, ensure_ascii=False, indent=2) + '\n')
hashes = json.loads((OUT.parent / 'sources_sha256.json').read_text())
def sha(p):
    h = hashlib.sha256()
    with Path(p).open('rb') as f:
        for b in iter(lambda: f.read(1024*1024), b''):
            h.update(b)
    return h.hexdigest()
assert all(sha(p) == value for p, value in hashes.items())
(OUT / 'validation.json').write_text(json.dumps(dict(original_batch_hashes_preserved=True,
    selected_grid_frames=10, retained_grid_cells=10*128*128, accumulation=False,
    operating_configuration_modified=False, grids_saved='selected frames only; not a decoded bag cache'), indent=2) + '\n')
(OUT / 'README_KO.txt').write_text('''시나리오1 두 의심 구간 BEV 확대
원본: 20261012_215236_scenario1_9e41f2. 사용자 직전 최신 그림의 두 구간으로 해석.
구간1: y≈-60~-30m 굴곡; 각 회차 계획 pose y≈-62m 시점의 최신 선행 BEV 선택.
구간2: y≈-30~-5m 회전; 각 회차 계획 pose y≈-30m 시점의 최신 선행 BEV 선택.
각 그림의 첫 패널은 배경 없는 원래 궤적 확대, 나머지 다섯 패널은 회차별 단일 BEV.
단일 BEV를 기록된 계획 pose의 map 위치/scalar yaw로 회전·이동. grid origin은 quaternion.
회색=비주행 셀, 흰색=주행 가능 셀, 옅은 청회색=BEV 범위 밖. unknown은 별도 황색.
지도 누적 및 모든 grid 디코딩 없음. 선택한 총10장 원시 셀 데이터160KiB만 보관.
BEV header 시각0, 계획에 사용한 grid ID 없음. 센서 획득 시각/정확한 입력 메시지 확정 불가.
뒤에 겹친 실제 궤적은 해당 구간 통과 전체 기록이며 정지된 배경과 시각이 다름.
이 그림으로 독립적인 도로 침범/차체 경계/획득 지연을 판정하지 않음.
원본/미커밋 코드/제어·PX4 설정 변경 없음. ROS 노드 실행·주행·ARM 없음.
''')
