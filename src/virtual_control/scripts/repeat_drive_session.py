#!/usr/bin/env python3
"""Own only this session's processes; never arm, set PX4 parameters or open serial."""
import argparse
from collections import deque
from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import socket
import subprocess
import threading
import time
import uuid

import yaml
from drive_debug_profile import PACKAGE, REQUIRED_TOPICS, TOPICS, recorder_command

WORKSPACE = PACKAGE.parents[1]
MIB = 1024**2
MIN_FREE = 1024*MIB
MAX_SESSION = 512*MIB
OWNED_NAMES = {'px4_ekf_bridge', 'px4_odom_map_bridge', 'knu_global_path_publisher',
               'upper_planner_node', 'lower_tracking_mpc_node', 'knu_batch_runner',
               'rosbag2_recorder', 'repeat_drive_session'}
SCENARIOS = tuple(json.loads((PACKAGE/'data/knu_routes/knu_batch_profile.json').read_text())['routes'])
PHASE_LABELS = {'startup': '입력·기록 준비', 'stopping': '출발 전 정지 확인',
                'pausing': '회차 전환 준비', 'aligning': '출발점 정렬',
                'map': '새 지도 대기', 'resetting': '새 회차 상태 초기화',
                'reference': '경로 준비', 'running': '주행 중',
                'coasting': '회차 종료·정지 대기', 'aborting': '중단·정지 대기',
                'aborted': '중단됨', 'done': '전체 완료'}


def effective_config(count, results, scenario='scenario3', guarded=False, config_file=None):
    if scenario not in (*SCENARIOS, 'all'):
        raise ValueError(f'Unsupported repeat scenario: {scenario}')
    config = {}
    # A sweep passes a complete frozen configuration; ordinary runs retain the
    # established base + manual-operation overlays.
    paths = ([Path(config_file)] if config_file is not None else
             [PACKAGE/'config'/name for name in
              ('tracking_control_split.yaml', 'knu_batch.yaml', 'scenario3_repeat.yaml')])
    for path in paths:
        document = yaml.safe_load(path.read_text())
        if not isinstance(document, dict):
            raise ValueError(f'{path}: expected a node parameter mapping')
        for node, section in document.items():
            config.setdefault(node, {'ros__parameters': {}})['ros__parameters'].update(
                section['ros__parameters'])
    for node in ('upper_planner_node', 'lower_tracking_mpc_node', 'knu_batch_runner',
                 'px4_ekf_bridge', 'px4_odom_map_bridge', 'knu_global_path_publisher'):
        if node not in config:
            raise ValueError(f'configuration is missing {node}')
    runner = config['knu_batch_runner']['ros__parameters']
    upper = config['upper_planner_node']['ros__parameters']
    # Outcome evidence uses the identical value convention/frame/bias as the planner.
    runner.update(enable_road_route_check=not upper.get('grid_origin_x_override_enabled',False),
                  road_grid_positive_is_drivable=upper.get('grid_positive_is_drivable',False),
                  road_grid_value_threshold=upper.get('grid_value_threshold',0),
                  road_expected_grid_frame=upper.get('expected_grid_frame_id','Vehicle'),
                  road_enable_waypoint_bias=upper.get('enable_waypoint_bias',False),
                  road_waypoint_yaw_bias_rad=upper.get('waypoint_yaw_bias_rad',0.0),
                  road_waypoint_x_bias_m=upper.get('waypoint_x_bias_m',0.0),
                  road_waypoint_y_bias_m=upper.get('waypoint_y_bias_m',0.0))
    routes = list(SCENARIOS) if scenario == 'all' else [scenario]
    runner.update(routes=routes, repeat_count=count, results_directory=str(results))
    if scenario == 'all' or guarded:
        runner.update(repeat_per_route=True, skip_system_fault_scenarios=True,
                      require_compute_heartbeat=True,
                      upper_compute_timeout_sec=5.0, lower_compute_timeout_sec=2.0)
    for name in ('px4_odom_map_bridge', 'knu_global_path_publisher'):
        config[name]['ros__parameters']['route_name'] = routes[0]
    return config


def directory_size(path):
    return sum(p.stat().st_size for p in path.rglob('*') if p.is_file())


def resource_error(free, used):
    if free < MIN_FREE:
        return '남은 공간이 1 GiB 미만입니다.'
    if used >= MAX_SESSION:
        return '이번 세션 저장량이 512 MiB에 도달했습니다.'
    return ''


class Child:
    """Drain console output without letting repeated warnings fill the disk."""
    def __init__(self, command, destination, env):
        self.destination = destination
        self.tail = deque(maxlen=15)
        self.process = subprocess.Popen(command, stdout=subprocess.PIPE,
                                        stderr=subprocess.STDOUT, start_new_session=True,
                                        env=env)
        self.thread = threading.Thread(target=self.drain, daemon=True)
        self.thread.start()

    def drain(self):
        with self.destination.open('wb') as stream:
            for line in iter(self.process.stdout.readline, b''):
                self.tail.append(line.decode(errors='replace').rstrip())
                if stream.tell()+len(line) > 2*MIB:
                    stream.flush()
                    shutil.copyfile(self.destination, self.destination.with_suffix('.previous.log'))
                    stream.seek(0)
                    stream.truncate()
                stream.write(line)
                stream.flush()

    def stop(self):
        if self.process.poll() is None:
            os.killpg(self.process.pid, signal.SIGINT)
            try:
                self.process.wait(timeout=12)
            except subprocess.TimeoutExpired:
                os.killpg(self.process.pid, signal.SIGTERM)
                try:
                    self.process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(self.process.pid, signal.SIGKILL)
                    self.process.wait(timeout=5)
        self.thread.join(timeout=2)


def preflight(node):
    import rclpy
    # DDS discovery needs time even when another terminal already owns the nodes.
    until = time.monotonic()+3.0
    while time.monotonic() < until:
        rclpy.spin_once(node, timeout_sec=0.1)
    others = [name for name, _ in node.get_node_names_and_namespaces()
              if name in OWNED_NAMES and name != node.get_name()]
    if others:
        raise RuntimeError('기존 5개 실행 터미널과 별도 bag 기록을 먼저 종료하세요: '+
                           ', '.join(sorted(set(others))))
    for port in (14540, 14541):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            try:
                sock.bind(('0.0.0.0', port))
            except OSError as error:
                raise RuntimeError(f'UDP {port}를 다른 실행기가 사용 중입니다.') from error
    free = shutil.disk_usage(WORKSPACE).free
    if free < MIN_FREE+MAX_SESSION:
        raise RuntimeError('시작하려면 여유 공간이 최소 1.5 GiB 필요합니다.')


def main():
    parser = argparse.ArgumentParser(description='Profile routes repeated SIH attempts with compressed logs')
    parser.add_argument('--scenario', choices=(*SCENARIOS, 'all'), default='scenario3')
    parser.add_argument('--runs', type=int, default=5, help="Attempt count (default: 5)")
    parser.add_argument('--guarded', action='store_true',
                        help='Enable 5s/2s compute monitoring and skip remaining runs '
                             'after system-fault stop and recovery, including single scenarios')
    parser.add_argument('--check', action='store_true', help='Read-only readiness check; no driving nodes')
    parser.add_argument('--config-file', type=Path, help='Complete prepared session configuration')
    parser.add_argument('--session-dir', type=Path, help='New explicit recording directory; must not exist')
    args = parser.parse_args()
    if not 1 <= args.runs <= 100:
        parser.error('--runs must be in [1, 100]')
    # Validate prepared configuration before creating a ROS node or children.
    try:
        effective_config(args.runs, Path('/tmp/repeat_config_validation'),
                         args.scenario, guarded=args.guarded, config_file=args.config_file)
        if args.session_dir is not None and args.session_dir.exists():
            raise ValueError(f'session directory already exists: {args.session_dir}')
    except (ValueError, OSError, KeyError) as error:
        parser.error(str(error))
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from std_msgs.msg import Bool, String
    rclpy.init()
    node = Node('repeat_drive_session')
    recorder = controller = None
    ready_pub = abort_pub = None
    session = None
    result = 1
    status = {}
    status_at = -float('inf')
    stopping = False
    stop_requested = None
    old_handlers = {}
    try:
        preflight(node)
        if args.check:
            print('점검 통과: 기존 제어기/기록기 충돌 없음, UDP 포트 및 저장공간 준비됨.')
            return 0
        session = (args.session_dir.resolve() if args.session_dir is not None else
                   WORKSPACE/'drive_batches'/(
                       datetime.now().strftime('%Y%m%d_%H%M%S_')+args.scenario+'_'+uuid.uuid4().hex[:6]))
        session.mkdir(parents=True)
        config = effective_config(args.runs, session/'results', args.scenario,
                                  guarded=args.guarded, config_file=args.config_file)
        runner_settings = config['knu_batch_runner']['ros__parameters']
        routes = runner_settings['routes']
        (session/'effective_config.yaml').write_text(yaml.safe_dump(config, sort_keys=False))
        sources = [PACKAGE/'config/tracking_control_split.yaml',
                   PACKAGE/'config/scenario3_repeat.yaml', Path(__file__),
                   PACKAGE/'scripts/knu_batch_runner.py', PACKAGE/'config/knu_batch.yaml',
                   PACKAGE/'scripts/knu_batch_monitor.py',
                   PACKAGE/'data/knu_routes/knu_batch_profile.json',
                   PACKAGE/'data/knu_routes/knu_start_poses.json']
        sources += [PACKAGE/f'data/knu_routes/{route}.csv' for route in routes]
        if args.config_file is not None:
            sources.append(args.config_file.resolve())
        # Snapshot the actual route definitions; later edits must not obscure old runs.
        route_snapshot = session/'routes'
        route_snapshot.mkdir()
        for path in sources:
            if path.parent.name == 'knu_routes':
                shutil.copy2(path, route_snapshot/path.name)
        sources += [WORKSPACE/'install/virtual_control/lib/virtual_control'/n for n in
                    ('upper_planner_node', 'lower_tracking_mpc_node', 'px4_odom_map_bridge')]
        (session/'manifest.json').write_text(json.dumps({
            'domain': os.environ.get('ROS_DOMAIN_ID'), 'runs': args.runs,
            'scenario': args.scenario,
            'routes': routes, 'planned_attempts': len(routes)*args.runs,
            'repeat_per_route': runner_settings.get('repeat_per_route', False),
            'skip_system_fault_scenarios': runner_settings.get('skip_system_fault_scenarios', False),
            'require_compute_heartbeat': runner_settings.get('require_compute_heartbeat', False),
            'upper_compute_timeout_sec': runner_settings.get('upper_compute_timeout_sec', 5.0),
            'lower_compute_timeout_sec': runner_settings.get('lower_compute_timeout_sec', 2.0),
            'topics': TOPICS, 'format': 'MCAP Zstd lossless',
            'max_session_bytes': MAX_SESSION, 'min_free_bytes': MIN_FREE,
            'sha256': {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources},
        }, indent=2)+'\n')
        env = dict(os.environ, ROS_LOG_DIR=str(session/'ros_logs'))
        ready_pub = node.create_publisher(Bool, '/knu_batch/recorder_ready', 1)
        abort_pub = node.create_publisher(String, '/knu_batch/abort', 1)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)

        def on_status(msg):
            nonlocal status, status_at
            try:
                value = json.loads(msg.data)
                if value.get('results', '').startswith(str(session)+'/'):
                    status, status_at = value, time.monotonic()
            except (ValueError, AttributeError):
                pass

        node.create_subscription(String, '/knu_batch/status', on_status, qos)

        def interrupt(signum, frame):
            nonlocal stopping, stop_requested
            stopping = True
            if stop_requested is None:
                stop_requested = time.monotonic()
                print('\n중단 요청: 주행 허용을 끄고 정지를 기다립니다. 즉시 제동 명령은 아닙니다.', flush=True)

        for sig in (signal.SIGINT, signal.SIGTERM):
            old_handlers[sig] = signal.signal(sig, interrupt)
        recorder = Child(recorder_command(session/'bag'), session/'recorder.log', env)
        controller = Child(['ros2', 'launch', str(PACKAGE/'launch/scenario3_repeat.launch.py'),
                            'config_file:='+str(session/'effective_config.yaml')],
                           session/'controllers.log', env)
        print(f'{", ".join(routes)} × 각각 {args.runs}회 / 저장: {session}', flush=True)
        started = time.monotonic()
        last_check = -float('inf')
        recorder_ready = False
        failure = ''
        previous_phase = None
        last_bag_size = 0
        bag_progress_at = started
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            now = time.monotonic()
            if now-last_check >= 0.5:
                last_check = now
                failure = failure or resource_error(shutil.disk_usage(session).free,
                                                     directory_size(session))
                if recorder.process.poll() is not None:
                    failure = failure or '기록기가 종료되었습니다.'
                if controller.process.poll() is not None:
                    failure = failure or '제어기 실행이 종료되었습니다.'
                if now-started > 15 and now-status_at > 5:
                    failure = failure or '반복 실행 상태 수신이 끊겼습니다.'
                bag_size = directory_size(session/'bag')
                if bag_size != last_bag_size:
                    last_bag_size, bag_progress_at = bag_size, now
                if status.get('phase') == 'running' and now-bag_progress_at > 15:
                    failure = failure or '기록 파일이 15초 이상 갱신되지 않았습니다.'
                recorder_ready = all(any(s.node_name == 'rosbag2_recorder' for s in
                                         node.get_subscriptions_info_by_topic(topic))
                                     for topic in REQUIRED_TOPICS)
            if failure and not stopping:
                print(f'자동 중단: {failure}', flush=True)
                stopping, stop_requested = True, now
            ready_pub.publish(Bool(data=recorder_ready and not stopping))
            if stopping:
                abort_pub.publish(String(data=failure or 'user interrupted'))
            key = (status.get('index'), status.get('phase'))
            if key != previous_phase and status:
                previous_phase = key
                last = status.get('last_result') or {}
                print(f"[{status['index']}/{status['total']}] "
                      f"{status.get('route', '')} "
                      f"{PHASE_LABELS.get(status['phase'], status['phase'])}"
                      f"  {last.get('outcome', '')}", flush=True)
            if status.get('phase') in ('done', 'aborted'):
                result = 0 if status['phase'] == 'done' else 1
                break
            if stopping and now-stop_requested > 65:
                failure = failure or '정지 확인 대기 초과; 수동으로 차량 상태를 확인하세요.'
                print(failure, flush=True)
                break
        (session/'session_result.json').write_text(json.dumps({
            'status': status, 'failure': failure, 'bytes': directory_size(session)}, indent=2)+'\n')
    except Exception as error:
        print(f'실행하지 못했습니다: {error}', flush=True)
    finally:
        if ready_pub is not None and rclpy.ok():
            ready_pub.publish(Bool(data=False))
            abort_pub.publish(String(data='session supervisor exiting'))
            until = time.monotonic()+1.2
            while time.monotonic() < until:
                rclpy.spin_once(node, timeout_sec=0.1)
        if controller:
            controller.stop()
        if recorder:
            recorder.stop()
        if session:
            print(f'기록 저장 완료: {session} ({directory_size(session)/MIB:.1f} MiB)', flush=True)
        for sig, handler in old_handlers.items():
            signal.signal(sig, handler)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return result


if __name__ == '__main__':
    raise SystemExit(main())
