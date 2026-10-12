#!/usr/bin/env python3
"""Run the selected KNU routes on SIH, reanchoring map coordinates between attempts."""

import csv
from datetime import datetime
import json
import math
from pathlib import Path
import time
import uuid

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid, Path as PathMsg
from rcl_interfaces.srv import SetParametersAtomically
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String, Float64MultiArray
from std_srvs.srv import Trigger

from knu_batch_monitor import Limits, RouteMonitor, RoadLimits, RoadRouteCheck
from knu_global_path_publisher import load_route, load_start_positions


# Driving outcomes (including stopped_10s/no_progress) do not skip repetitions.
SYSTEM_FAULT_OUTCOMES = frozenset({'pose_timeout', 'grid_timeout', 'setup_timeout',
                                  'alignment_failed', 'upper_compute_timeout',
                                  'lower_compute_timeout'})


class KnuBatchRunner(Node):
    """Bounded setup/run/stop states; run permission expires in the lower node."""

    def __init__(self, **kwargs):
        super().__init__('knu_batch_runner', **kwargs)
        directory = Path(get_package_share_directory('virtual_control')) / 'data/knu_routes'
        batch_routes = json.loads((directory/'knu_batch_profile.json').read_text())['routes']
        defaults = {
            'routes': batch_routes,
            'odom_topic': '/motive/vehicle/odom_map',
            'grid_topic': '/bev/occupancy_grid',
            'global_path_topic': '/navigation/global_path',
            'reference_topic': '/planner/lower_reference_path',
            'require_matlab_ack': False,
            'repeat_count': 1,
            'repeat_per_route': False,
            'skip_system_fault_scenarios': False,
            'require_compute_heartbeat': False,
            'upper_compute_timeout_sec': 5.0,
            'lower_compute_timeout_sec': 2.0,
            'require_recorder': False,
            'reset_upper_each_attempt': False,
            'recorder_timeout_sec': 2.0,
            'stationary_timeout_sec': 60.0,
            'startup_timeout_sec': 120.0, 'setup_timeout_sec': 30.0,
            'pose_loss_sec': 10.0, 'grid_loss_sec': 10.0,
            'pose_fresh_sec': 0.6, 'grid_fresh_sec': 2.0,
            'results_directory': str(Path.cwd() / 'knu_batch_results'),
            'enable_road_route_check': True,
            'road_grid_positive_is_drivable': False,
            'road_grid_value_threshold': 0,
            'road_expected_grid_frame': 'Vehicle',
            # Existing Isaac BEV messages have zero header stamps; use reception age.
            'road_allow_unstamped_grids': True,
            'road_enable_waypoint_bias': False,
            'road_waypoint_yaw_bias_rad': 0.0,
            'road_waypoint_x_bias_m': 0.0,
            'road_waypoint_y_bias_m': 0.0,
        }
        defaults.update(vars(Limits()))
        defaults.update(vars(RoadLimits()))
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.settings = {name: self.get_parameter(name).value for name in defaults}
        for key, value in self.settings.items():
            if (isinstance(value, float) and not key.startswith('road_waypoint_')
                    and (not math.isfinite(value) or value <= 0)):
                raise ValueError(f'{key} must be positive and finite')
        self.limits = Limits(**{k: self.settings[k] for k in vars(Limits())})
        self.road_limits = RoadLimits(**{k:self.settings[k] for k in vars(RoadLimits())})
        if not 0 <= self.settings['road_grid_value_threshold'] <= 100:
            raise ValueError('road_grid_value_threshold must be in [0,100]')
        if not self.settings['road_expected_grid_frame'].lstrip('/'):
            raise ValueError('road_expected_grid_frame must be nonempty')
        if any(not math.isfinite(self.settings[k]) for k in
               ('road_waypoint_yaw_bias_rad','road_waypoint_x_bias_m','road_waypoint_y_bias_m')):
            raise ValueError('road waypoint bias must be finite')
        self.routes = self.settings['routes']
        if not self.routes or len(set(self.routes)) != len(self.routes):
            raise ValueError('routes must be a nonempty list without duplicates')
        count = self.settings['repeat_count']
        if type(count) is not int or not 1 <= count <= 100:
            raise ValueError('repeat_count must be an integer in [1, 100]')
        self.routes = ([route for route in self.routes for _ in range(count)]
                       if self.settings['repeat_per_route'] else self.routes * count)
        self.system_fault_scenarios = {}
        directory = Path(get_package_share_directory('virtual_control')) / 'data/knu_routes'
        self.points = {name: load_route(name, directory) for name in self.routes}
        self.start_positions = load_start_positions(directory)
        self.qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.permission_pub = self.create_publisher(Bool, '/knu_batch/run_permission', 1)
        self.path_pub = self.create_publisher(
            PathMsg, self.settings['global_path_topic'], self.qos)
        self.route_pub = self.create_publisher(String, '/knu_batch/active_route', self.qos)
        self.status_pub = self.create_publisher(String, '/knu_batch/status', self.qos)
        self.reset_pub = self.create_publisher(Bool, '/controller/reset', 10)
        self.create_subscription(Odometry, self.settings['odom_topic'], self.on_pose, 10)
        self.create_subscription(OccupancyGrid, self.settings['grid_topic'], self.on_grid, 10)
        self.create_subscription(PathMsg, self.settings['reference_topic'], self.on_reference,
                                 self.qos)
        self.create_subscription(String, '/knu_batch/grid_ready', self.on_ack, 10)
        self.create_subscription(String, '/knu_batch/logging_paused', self.on_logging_paused, 10)
        diagnostic_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(String, '/debug/upper_stage_timing', self.on_upper_cycle,
                                 diagnostic_qos)
        self.create_subscription(Float64MultiArray, '/debug/lower_mpc_trace', self.on_lower_cycle,
                                 diagnostic_qos)
        self.upper_cycle_at = self.lower_cycle_at = -math.inf
        self.upper_cycle_seq = -1
        self.lower_cycle_stamp = -math.inf
        self.compute_overrun = None
        self.create_subscription(Bool, '/knu_batch/recorder_ready', self.on_recorder, 1)
        self.create_subscription(String, '/knu_batch/abort',
                                 lambda msg: self.abort(msg.data), 1)
        self.recorder_at = -math.inf
        self.recorder_ok = False
        self.recorder_seen = False
        self.client = self.create_client(
            SetParametersAtomically, '/px4_odom_map_bridge/set_parameters_atomically')
        self.upper_reset_client = self.create_client(Trigger, '/upper_planner_node/reset_mission')
        self.upper_reset_future = None
        self.reference_not_before_ns = 0
        self.pose = None
        self.pose_yaw = None
        self.pose_stamp_sec = None
        self.road_check = None
        self.road_sample_at = -math.inf
        self.road_grid_sequence = 0
        self.road_grid_stamp = None
        self.pose_at = self.grid_at = self.reference_at = -math.inf
        self.pause_ack_token = None
        self.logger_required = False
        self.ack = None
        self.ack_at = -math.inf
        self.index = -1
        self.route = ''
        self.token = ''
        self.monitor = None
        self.future = None
        self.stationary_since = None
        self.phase = 'startup'
        self.entered = time.monotonic()
        self.attempt_started = self.entered
        self.session = uuid.uuid4().hex[:10]
        self.phase_sequence = 0
        self.phase_stamp_sec = self.get_clock().now().nanoseconds * 1e-9
        self.last_status_at = -math.inf
        self.last_result = None
        self.rows = []
        out = Path(self.settings['results_directory']).expanduser()
        self.output = out / (datetime.now().strftime('%Y%m%d_%H%M%S_') + self.session)
        self.output.mkdir(parents=True, exist_ok=False)
        (self.output / 'settings.json').write_text(json.dumps(self.settings, indent=2)+'\n')
        self.fields = ['route', 'token', 'outcome', 'duration_sec', 'progress_ratio',
                       'best_progress_ratio', 'cross_track_m', 'goal_distance_m', 'detail',
                       'stop_status', 'stop_x', 'stop_y', 'stop_goal_distance_m']
        self.publish_permission(False)
        self.publish_status()
        self.timer = self.create_timer(0.1, self.tick)
        self.get_logger().info(f'{len(self.routes)} SIH routes; results: {self.output}')

    def on_pose(self, msg):
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        if msg.header.frame_id.lstrip('/') != 'map':
            return
        values = (p.x, p.y, p.z, v.x, v.y)
        if all(math.isfinite(x) for x in values):
            self.pose = (p.x, p.y, math.hypot(v.x, v.y))
            self.pose_at = time.monotonic()
            # This project's odometry stores scalar yaw directly in orientation.z.
            yaw = msg.pose.pose.orientation.z
            self.pose_yaw = yaw if math.isfinite(yaw) else None
            self.pose_stamp_sec = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9

    def on_recorder(self, msg):
        self.recorder_at = time.monotonic()
        self.recorder_ok = msg.data
        self.recorder_seen = self.recorder_seen or msg.data

    def on_upper_cycle(self, msg):
        try:
            value = json.loads(msg.data)
            sequence = value['cycle_seq']
            if type(sequence) is int and sequence > self.upper_cycle_seq:
                self.upper_cycle_seq = sequence
                self.upper_cycle_at = time.monotonic()
                duration = value.get('wall_ms')
                if (self.phase == 'running' and isinstance(duration, (int, float)) and
                        math.isfinite(duration) and
                        duration >= 1000*self.settings['upper_compute_timeout_sec']):
                    self.compute_overrun = 'upper_compute_timeout'
        except (ValueError, KeyError, TypeError):
            pass

    def on_lower_cycle(self, msg):
        if (len(msg.data) >= 27 and math.isfinite(msg.data[0]) and
                msg.data[0] > self.lower_cycle_stamp):
            self.lower_cycle_stamp = msg.data[0]
            self.lower_cycle_at = time.monotonic()
            # Trace[11] measures build-to-solver-return, not the complete lower callback.
            if (self.phase == 'running' and math.isfinite(msg.data[11]) and
                    msg.data[11] >= 1000*self.settings['lower_compute_timeout_sec']):
                self.compute_overrun = 'lower_compute_timeout'

    def compute_issue(self, now):
        if self.settings['require_compute_heartbeat']:
            if self.compute_overrun:
                return self.compute_overrun
            for level in ('upper', 'lower'):
                age = now-getattr(self, level+'_cycle_at')
                if age >= self.settings[level+'_compute_timeout_sec']:
                    return level+'_compute_timeout'
        return None

    def abort(self, reason):
        if self.phase in ('done', 'aborting', 'aborted'):
            return
        if self.route and not any(row['token'] == self.token for row in self.rows):
            self.record('batch_aborted', reason)
        (self.output / 'abort.json').write_text(json.dumps({
            'reason': reason, 'phase': self.phase, 'token': self.token}, indent=2)+'\n')
        self.publish_permission(False)
        self.stationary_since = None
        self.set_phase('aborting')
        self.get_logger().error(f'Batch aborted: {reason}; waiting for stationary pose')

    def on_grid(self, msg):
        # Discard malformed or empty maps; normal drivable convention belongs to the planner.
        if (msg.info.width > 0 and msg.info.height > 0 and msg.info.resolution > 0
                and len(msg.data) == msg.info.width * msg.info.height):
            self.grid_at = time.monotonic()
        now = time.monotonic()
        if (self.phase != 'running' or self.road_check is None
                or now-self.road_sample_at < 0.2):
            return
        self.road_sample_at = now
        self.road_check.expire(now)
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if stamp > 0:
            if self.road_grid_stamp is not None and stamp <= self.road_grid_stamp:
                return
            self.road_grid_stamp = stamp
        self.road_grid_sequence += 1
        synchronized = (abs(stamp-self.pose_stamp_sec) <= self.road_limits.road_pose_skew_sec
                        if stamp > 0 and self.pose_stamp_sec is not None else
                        stamp == 0 and self.settings['road_allow_unstamped_grids']
                        and now-self.pose_at <= self.road_limits.road_pose_skew_sec)
        if (not synchronized or self.pose_stamp_sec is None or self.pose_yaw is None
                or now-self.pose_at > self.settings['pose_fresh_sec']):
            evidence = {'state':'unavailable','reason':'missing or unsynchronized grid/pose stamps'}
        else:
            evidence = self.road_check.sample(
                msg,self.pose[:2],self.pose_yaw,self.monitor.progress,
                self.settings['road_grid_positive_is_drivable'],
                self.settings['road_grid_value_threshold'],
                self.settings['road_expected_grid_frame'])
        evidence.update(clock_source='header' if stamp > 0 else 'receive_monotonic',
                        acquisition_stamp_available=stamp > 0)
        self.road_check.observe(now,self.road_grid_sequence,evidence)

    def on_reference(self, msg):
        stamp = msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec
        if (len(msg.poses) >= 2 and msg.header.frame_id.lstrip('/') == 'map'
                and stamp >= self.reference_not_before_ns):
            self.reference_at = time.monotonic()

    def on_ack(self, msg):
        try:
            value = json.loads(msg.data)
            if value.get('token') == self.token:
                self.ack = value
                self.ack_at = time.monotonic()
        except (ValueError, AttributeError):
            pass

    def on_logging_paused(self, msg):
        try:
            value = json.loads(msg.data)
            if value.get('token') == self.token and value.get('paused') is True:
                self.pause_ack_token = self.token
        except (ValueError, AttributeError):
            pass

    def set_phase(self, phase):
        self.phase, self.entered = phase, time.monotonic()
        self.phase_sequence += 1
        self.phase_stamp_sec = self.get_clock().now().nanoseconds * 1e-9
        self.publish_status()

    def publish_status(self):
        self.last_status_at = time.monotonic()
        self.status_pub.publish(String(data=json.dumps({
            'phase': self.phase, 'route': self.route, 'index': min(self.index+1, len(self.routes)),
            'total': len(self.routes), 'results': str(self.output),
            'session': self.session, 'token': self.token, 'sequence': self.phase_sequence,
            'phase_stamp_sec': self.phase_stamp_sec, 'last_result': self.last_result})))

    def publish_permission(self, allowed):
        self.permission_pub.publish(Bool(data=allowed))

    def next_route(self):
        self.publish_permission(False)
        self.index += 1
        # Called only after a fresh stationary pose, never while coasting.
        while self.index < len(self.routes) and self.routes[self.index] in self.system_fault_scenarios:
            route = self.routes[self.index]
            row = dict.fromkeys(self.fields, '')
            row.update(route=route, token=f'{self.session}:{self.index}:{route}',
                       outcome='skipped_system_fault', duration_sec=0.0,
                       detail=self.system_fault_scenarios[route])
            self.rows.append(row)
            self.last_result = row.copy()
            self.write_summary()
            self.get_logger().warn(f'[{self.index+1}/{len(self.routes)}] {route}: skipped; {row["detail"]}')
            self.index += 1
        if self.index == len(self.routes):
            self.set_phase('done')
            self.route_pub.publish(String(data=json.dumps({'done': True, 'token': self.token})))
            self.get_logger().info(f'Batch finished; results: {self.output / "summary.csv"}')
            return
        self.route = self.routes[self.index]
        self.token = f'{self.session}:{self.index}:{self.route}'
        self.pause_ack_token = None
        self.logger_required = bool(self.settings['require_matlab_ack'])
        self.monitor = None
        self.road_check = None
        self.ack = None
        self.future = None
        self.upper_reset_future = None
        self.stationary_since = None
        self.attempt_started = time.monotonic()
        self.set_phase('stopping')
        self.get_logger().info(f'[{self.index+1}/{len(self.routes)}] {self.route}')

    def record(self, outcome, detail=''):
        self.publish_permission(False)
        row = dict.fromkeys(self.fields, '')
        row.update(route=self.route, token=self.token, outcome=outcome,
                   duration_sec=round(time.monotonic()-self.attempt_started, 3), detail=detail)
        if self.monitor:
            row.update({k: round(v, 4) if math.isfinite(v) else ''
                        for k, v in self.monitor.snapshot().items()})
        self.rows.append(row)
        self.last_result = row.copy()
        self.write_summary()
        self.get_logger().info(f'{self.route}: {outcome} {detail}')

    def write_summary(self):
        # Rewrite atomically after every route, so interrupted runs retain completed results.
        temporary = self.output / 'summary.csv.tmp'
        with temporary.open('w', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=self.fields)
            writer.writeheader()
            writer.writerows(self.rows)
        temporary.replace(self.output / 'summary.csv')

    def fail_or_finish(self, outcome, detail=''):
        self.record(outcome, detail)
        self.compute_overrun = None
        if (self.settings['skip_system_fault_scenarios'] and
                outcome in SYSTEM_FAULT_OUTCOMES):
            self.system_fault_scenarios[self.route] = f'{self.token}: {outcome} {detail}'.strip()
        self.stationary_since = None
        self.set_phase('coasting')

    def record_stop(self, stopped):
        if not self.rows or self.rows[-1]['token'] != self.token:
            return
        row = self.rows[-1]
        row['stop_status'] = 'stationary' if stopped else 'timeout'
        if stopped:
            row.update(stop_x=round(self.pose[0], 4), stop_y=round(self.pose[1], 4),
                       stop_goal_distance_m=round(math.dist(
                           self.pose[:2], self.points[self.route][-1][:2]), 4))
        self.last_result = row.copy()
        self.write_summary()

    def send_path(self):
        msg = PathMsg()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        for x, y, z in self.points[self.route]:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = x, y, z
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        self.path_pub.publish(msg)

    def tick(self):
        now = time.monotonic()
        if now-self.last_status_at >= 0.5:
            self.publish_status()
        pose_fresh = now-self.pose_at <= self.settings['pose_fresh_sec']
        grid_fresh = now-self.grid_at <= self.settings['grid_fresh_sec']
        recorder_fresh = (not self.settings['require_recorder'] or
                          (self.recorder_ok and now-self.recorder_at <=
                           self.settings['recorder_timeout_sec']))
        compute_issue = self.compute_issue(now)
        self.publish_permission(self.phase == 'running' and pose_fresh and grid_fresh
                                and recorder_fresh and not compute_issue)
        if self.phase in ('done', 'aborted'):
            return
        if (self.settings['require_recorder'] and self.recorder_seen and not recorder_fresh
                and self.phase != 'aborting'):
            self.abort('recorder heartbeat lost or recording/storage unavailable')
        if self.phase == 'startup':
            ready = (self.client.service_is_ready() and pose_fresh and grid_fresh and not compute_issue
                     and recorder_fresh and self.permission_pub.get_subscription_count() >= 1
                     and (not self.settings['reset_upper_each_attempt'] or
                          self.upper_reset_client.service_is_ready()))
            if ready:
                self.next_route()
            elif now-self.entered >= self.settings['startup_timeout_sec']:
                self.abort('startup_timeout: pose/grid/bridge/lower/recorder not ready')
            return
        if self.phase in ('stopping', 'coasting', 'aborting'):
            stopped = pose_fresh and self.pose[2] < 0.2
            if (self.phase == 'coasting' and self.settings['skip_system_fault_scenarios']
                    and self.route not in self.system_fault_scenarios):
                stop_issue = (compute_issue or
                              ('pose_timeout' if now-self.pose_at >= self.settings['pose_loss_sec']
                               else 'grid_timeout' if now-self.grid_at >= self.settings['grid_loss_sec']
                               else None))
                if stop_issue:
                    detail = f'{self.token}: {stop_issue} during coasting'
                    self.system_fault_scenarios[self.route] = detail
                    self.rows[-1]['detail'] = (self.rows[-1]['detail']+'; '+detail).strip('; ')
                    self.last_result = self.rows[-1].copy()
                    self.write_summary()
            recovering = (self.phase == 'coasting' and
                          self.route in self.system_fault_scenarios)
            alignment_idle = self.future is None or self.future.done()
            self.stationary_since = ((now if self.stationary_since is None
                                      else self.stationary_since) if stopped else None)
            if (self.stationary_since is not None and now-self.stationary_since >= 1.0
                    and (not recovering or (grid_fresh and not compute_issue and alignment_idle))):
                if self.phase == 'stopping':
                    self.set_phase('pausing')
                else:
                    self.record_stop(True)
                    if self.phase == 'aborting':
                        self.set_phase('aborted')
                    else:
                        self.next_route()
            elif now-self.entered >= self.settings['stationary_timeout_sec']:
                confirmed_stop = (self.stationary_since is not None and
                                  now-self.stationary_since >= 1.0)
                self.record_stop(confirmed_stop)
                if self.phase != 'aborting':
                    self.abort('system_recovery_timeout: grid/compute/alignment still unavailable' if
                               recovering and confirmed_stop else
                               'stop_timeout: no fresh stationary pose; no further reanchor')
                self.set_phase('aborted')
            return
        if self.phase in ('pausing', 'aligning', 'map', 'resetting', 'reference'):
            if now-self.entered >= self.settings['setup_timeout_sec']:
                if self.phase == 'resetting':
                    self.abort('upper mission reset acknowledgement timeout')
                else:
                    self.fail_or_finish('setup_timeout', self.phase)
                return
        if self.phase == 'pausing':
            self.logger_required = (self.logger_required or
                                    self.count_publishers('/knu_batch/logging_paused') > 0)
            if not self.logger_required or self.pause_ack_token == self.token:
                self.set_phase('aligning')
            return
        if self.phase == 'aligning':
            if self.future is None:
                if not self.client.service_is_ready():
                    return
                # A second route owner could overwrite missions mid-run.
                if (self.count_publishers(self.settings['global_path_topic']) != 1 or
                        self.count_publishers(self.settings['odom_topic']) != 1):
                    self.abort('conflicting_publishers: stop the single-route launch')
                    return
                request = SetParametersAtomically.Request()
                request.parameters = [Parameter('route_name', value=self.route).to_parameter_msg()]
                self.future = self.client.call_async(request)
            elif self.future.done():
                try:
                    result = self.future.result().result
                    if not result.successful:
                        self.fail_or_finish('alignment_failed', result.reason)
                        return
                except Exception as error:
                    self.fail_or_finish('alignment_failed', str(error))
                    return
                self.set_phase('map')
                self.map_pose_ready_at = None
                self.route_pub.publish(String(data=json.dumps({
                    'route': self.route, 'token': self.token, 'done': False})))
            return
        if self.phase == 'map':
            anchor = self.start_positions[self.route][:2]
            aligned = (pose_fresh and self.pose_at > self.entered
                       and math.dist(self.pose[:2], anchor) < 2.0)
            if aligned and self.map_pose_ready_at is None:
                self.map_pose_ready_at = now
            if not aligned or self.map_pose_ready_at is None:
                return
            # Allow the external map provider to consume the newly aligned odometry.
            ready = (grid_fresh and self.grid_at > self.map_pose_ready_at + 1.0)
            if self.settings['require_matlab_ack']:
                ready = ready and self.ack is not None and self.ack_at > self.entered
            if ready:
                if self.settings['reset_upper_each_attempt']:
                    self.set_phase('resetting')
                    return
                self.reset_pub.publish(Bool(data=True))
                self.send_path()
                self.set_phase('reference')
            return
        if self.phase == 'resetting':
            if self.upper_reset_future is None:
                if self.upper_reset_client.service_is_ready():
                    self.upper_reset_future = self.upper_reset_client.call_async(Trigger.Request())
            elif self.upper_reset_future.done():
                try:
                    response = self.upper_reset_future.result()
                    if not response.success:
                        self.abort('upper mission reset rejected: '+response.message)
                        return
                except Exception as error:
                    self.abort('upper mission reset failed: '+str(error))
                    return
                # Queued or latched references from the preceding attempt cannot authorize driving.
                self.reference_at = -math.inf
                self.reference_not_before_ns = self.get_clock().now().nanoseconds
                self.get_logger().info(f'{self.token}: {response.message}')
                self.reset_pub.publish(Bool(data=True))
                self.send_path()
                self.set_phase('reference')
            return
        if self.phase == 'reference':
            if (self.reference_at > self.entered + 0.2 and pose_fresh and grid_fresh
                    and not compute_issue and now-self.entered > 1.0):
                self.reset_pub.publish(Bool(data=True))
                self.monitor = RouteMonitor(self.points[self.route], now, self.limits)
                if self.settings['enable_road_route_check']:
                    points = self.points[self.route]
                    if self.settings['road_enable_waypoint_bias']:
                        # Match the planner's first-waypoint-centred transform for evidence only.
                        angle = self.settings['road_waypoint_yaw_bias_rad']
                        c,s = math.cos(angle),math.sin(angle)
                        x0,y0 = points[0][:2]
                        points = [(x0+c*(x-x0)-s*(y-y0)+self.settings['road_waypoint_x_bias_m'],
                                   y0+s*(x-x0)+c*(y-y0)+self.settings['road_waypoint_y_bias_m'])
                                  for x,y,*_ in points]
                    self.road_check = RoadRouteCheck(points,self.road_limits)
                    self.road_sample_at = -math.inf
                    self.road_grid_sequence = 0
                    self.road_grid_stamp = None
                self.set_phase('running')
            return
        if self.phase == 'running':
            if compute_issue:
                self.fail_or_finish(compute_issue,
                                    'completion diagnostic gap or measured overrun; '
                                    f'upper_age={now-self.upper_cycle_at:.3f}s, '
                                    f'lower_age={now-self.lower_cycle_at:.3f}s, '
                                    f'measured_overrun={self.compute_overrun}; '
                                    'receive monotonic clock')
            elif now-self.pose_at >= self.settings['pose_loss_sec']:
                self.fail_or_finish('pose_timeout')
            elif now-self.grid_at >= self.settings['grid_loss_sec']:
                self.fail_or_finish('grid_timeout')
            elif now-self.monitor.started >= self.limits.route_timeout_sec:
                self.fail_or_finish('route_timeout')
            elif pose_fresh:
                result = self.monitor.update(now, self.pose[:2])
                waypoint_result = result
                if (self.road_check is not None and self.road_check.confirmed(now)
                        and result not in ('completed','pose_jump','invalid_pose','wrong_direction')):
                    result = self.monitor.outcome = 'route_unavailable'
                if result:
                    detail = (json.dumps({'outcome_policy':'road_route_v1',
                                          'waypoint_guard_result':waypoint_result,
                                          'road_evidence':self.road_check.detail(now)},sort_keys=True)
                              if self.road_check is not None else '')
                    self.fail_or_finish(result,detail)

    def stop(self):
        self.publish_permission(False)
        if (self.route and self.phase not in ('done', 'aborted') and
                not any(row['token'] == self.token for row in self.rows)):
            self.record('interrupted', self.phase)
        if self.phase != 'done':
            self.set_phase('aborted')
        # Keep the normal context alive briefly to deliver neutral permission on Ctrl+C.
        self.publish_status()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = KnuBatchRunner()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                node.stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
