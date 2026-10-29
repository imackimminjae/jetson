#!/usr/bin/env python3
"""Manually triggered straight throttle pulses, measured directly by Motive."""
import argparse
from collections import deque
import csv
from datetime import datetime
import json
import math
from pathlib import Path
import socket
import statistics
import time

from pymavlink.dialects.v20 import common as mav
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
import yaml


def actuator_message(duty):
    # Same command and Set 1/Set 2 mapping as lower_tracking_mpc_node.
    return mav.MAVLink_command_long_message(
        1, 1, 187, 0, duty, 0.0, math.nan, math.nan, math.nan, math.nan, 0.0)


class PoseVelocity:
    """Physical velocity over a 0.1 s window; reject reordered samples and gaps."""
    def __init__(self):
        self.samples = deque()

    def update(self, x, y, stamp):
        if not all(math.isfinite(value) for value in (x, y, stamp)):
            self.samples.clear()
            return None
        if self.samples and stamp <= self.samples[-1][0]:
            return None
        if self.samples and stamp - self.samples[-1][0] > .2:
            self.samples.clear()
        self.samples.append((stamp, x, y))
        while len(self.samples) > 2 and stamp - self.samples[1][0] >= .1:
            self.samples.popleft()
        oldest, ox, oy = self.samples[0]
        dt = stamp - oldest
        if dt < .08:
            return None
        return (x - ox) / dt, (y - oy) / dt


class Link:
    def __init__(self, port):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # Exclusive ownership: a running lower controller must not share this port.
        try:
            self.socket.bind(('0.0.0.0', port))
        except Exception:
            self.socket.close()
            raise
        self.socket.setblocking(False)
        self.peer = None
        self.codec = mav.MAVLink(self, srcSystem=245, srcComponent=190)
        self.codec.robust_parsing = True
        self.heartbeat_at = -math.inf
        self.armed = False
        self.main_mode = None
        self.pwm = {}
        self.pwm_at = -math.inf
        self.last_ack = None
        self.arm_ack = None
        self.last_statustext = ''
        self.text_chunks = None
        self.text_chunk_at = -math.inf
        self.rejected = False
        self.requested_pwm = False

    def write(self, data):
        return self.socket.sendto(data, self.peer) if self.peer else 0

    def poll(self, now):
        for _ in range(64):
            try:
                data, address = self.socket.recvfrom(65535)
            except BlockingIOError:
                break
            for message in self.codec.parse_buffer(data) or []:
                if message.get_srcSystem() != 1 or message.get_srcComponent() != 1:
                    continue
                kind = message.get_type()
                if kind == 'HEARTBEAT' and message.autopilot != mav.MAV_AUTOPILOT_INVALID:
                    self.peer = address
                    self.heartbeat_at = now
                    self.armed = bool(message.base_mode & mav.MAV_MODE_FLAG_SAFETY_ARMED)
                    self.main_mode = (int(message.custom_mode) >> 16) & 255
                elif kind == 'COMMAND_ACK' and message.command == 187:
                    self.last_ack = int(message.result)
                    self.rejected = message.result not in (mav.MAV_RESULT_ACCEPTED,
                                                           mav.MAV_RESULT_IN_PROGRESS)
                elif kind == 'COMMAND_ACK' and message.command == 400:
                    self.arm_ack = int(message.result)
                elif kind == 'STATUSTEXT':
                    chunk = str(message.text)
                    if message.chunk_seq == 0:
                        self.last_statustext = chunk
                        self.text_chunks = (message.id, 1)
                        self.text_chunk_at = now
                    elif (self.text_chunks == (message.id, message.chunk_seq)
                          and now - self.text_chunk_at <= 2):
                        self.last_statustext += chunk
                        self.text_chunks = (message.id, message.chunk_seq + 1)
                        self.text_chunk_at = now
                elif kind == 'SERVO_OUTPUT_RAW' and message.port == 0:
                    self.pwm = {i: getattr(message, f'servo{i}_raw', 0) for i in range(1, 17)}
                    self.pwm_at = now
        if self.peer and not self.requested_pwm:
            # Stream request only: no PX4 parameter, mode or ARM changes.
            self.codec.send(mav.MAVLink_command_long_message(
                1, 1, mav.MAV_CMD_SET_MESSAGE_INTERVAL, 0, 36, 50000, 0, 0, 0, 0, 0))
            self.requested_pwm = True

    def ready(self, now):
        return (self.peer is not None and now - self.heartbeat_at <= 3.0
                and self.armed and self.main_mode == 1)

    def send(self, duty):
        if self.peer:
            # Match lower_controller keepalive: neutral joystick, then actuator override.
            self.codec.send(mav.MAVLink_manual_control_message(1, 0, 0, 0, 0, 0))
            self.codec.send(actuator_message(duty))

    def request_manual_mode(self):
        if self.peer:
            self.send(0.0)
            self.codec.send(mav.MAVLink_set_mode_message(
                1, mav.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 1 << 16))

    def request_arm(self):
        if self.peer:
            self.send(0.0)
            self.arm_ack = None
            # Same normal ARM command as the lower controller; param2=0, no force.
            self.codec.send(mav.MAVLink_command_long_message(
                1, 1, mav.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0))

    def close(self):
        self.socket.close()


class Pulse:
    """One bounded pulse; every exit latches neutral until another explicit start."""
    def __init__(self, duty, duration, distance, speed):
        self.duty, self.duration = duty, duration
        self.distance_limit, self.speed_limit = distance, speed
        self.active = False
        self.started = None
        self.origin = None
        self.last_position = None
        self.distance = 0.0
        self.reason = ''

    def start(self, now, pose):
        self.active, self.started, self.origin = True, now, pose
        self.last_position = pose[:2]
        self.distance, self.reason = 0.0, ''

    def stop(self, reason):
        self.active, self.reason = False, reason
        return 0.0

    def output(self, now, pose, pose_at, ready, conflict=False):
        if not self.active:
            return 0.0
        if pose is None or now - pose_at > 0.20:
            return self.stop('motive_lost')
        if conflict:
            return self.stop('controller_conflict')
        if not ready:
            return self.stop('heartbeat_arm_or_command_fault')
        step = math.dist(pose[:2], self.last_position)
        self.distance += step
        self.last_position = pose[:2]
        dx, dy = pose[0] - self.origin[0], pose[1] - self.origin[1]
        lateral = abs(-math.sin(self.origin[2]) * dx + math.cos(self.origin[2]) * dy)
        heading = abs(math.atan2(math.sin(pose[2] - self.origin[2]),
                                 math.cos(pose[2] - self.origin[2])))
        if step > 0.15:
            return self.stop('pose_jump')
        if self.speed_limit is not None and pose[3] >= self.speed_limit:
            return self.stop('speed_limit')
        if (self.distance_limit is not None
                and self.distance >= self.distance_limit - 1e-9):
            return self.stop('distance_limit')
        if lateral >= 0.10 or heading >= math.radians(10):
            return self.stop('straight_line_deviation')
        if now - self.started >= self.duration:
            return self.stop('completed')
        return self.duty


def summarize(rows):
    """Keep short transient runs separate from a confirmed constant-speed interval."""
    moving = [row for row in rows if row['phase'] == 'pulse' and row['sent_duty'] > 0]
    tail_start = max(.35, moving[-1]['pulse_sec'] - 1.0) if moving else .35
    tail = [row for row in moving if row['pulse_sec'] >= tail_start]
    all_speeds = [row['speed_mps'] for row in moving]
    speeds = [row['speed_mps'] for row in tail]
    span = tail[-1]['pulse_sec'] - tail[0]['pulse_sec'] if len(tail) > 1 else 0.0
    slope = ((speeds[-1] - speeds[0]) / span) if span else None
    pwm = [row['reported_throttle_pwm_us'] for row in tail
           if row.get('reported_throttle_pwm_us')]
    # Use the same 0.03 m/s threshold as stationary detection. Motive noise at
    # rest is not evidence of a usable moving-speed calibration point.
    motion_detected = bool(speeds and statistics.median(speeds) >= .03)
    return {
        'samples': len(moving), 'tail_samples': len(tail),
        'mean_speed_mps': statistics.mean(all_speeds) if all_speeds else None,
        'peak_speed_mps': max(all_speeds) if all_speeds else None,
        'tail_median_speed_mps': statistics.median(speeds) if speeds else None,
        'tail_min_speed_mps': min(speeds) if speeds else None,
        'tail_max_speed_mps': max(speeds) if speeds else None,
        'tail_speed_slope_mps2': slope,
        'tail_reported_throttle_pwm_us': statistics.median(pwm) if pwm else None,
        'tail_reported_pwm_samples': len(pwm),
        'motion_detected': motion_detected,
        'steady_candidate': bool(motion_detected and len(tail) >= 8 and span >= 0.4
                                 and abs(slope) <= 0.05 and max(speeds) - min(speeds) <= 0.05),
    }


class Calibration(Node):
    def __init__(self, args):
        super().__init__('duty_calibration')
        self.args = args
        self.link = Link(args.port) if args.drive else None
        self.pose, self.pose_at = None, -math.inf
        self.pose_stamp_sec = None
        self.pose_velocity = PoseVelocity()
        self.stationary_at = None
        self.conflict = False
        self.graph_at = -math.inf
        self.index, self.trial, self.pulse = 0, 0, None
        self.stopping_at = None
        self.stop_distance = 0.0
        self.stop_position = None
        self.rows, self.results = [], []
        self.blocked_reason = ''
        self.arm_requested_at = None
        self.arm_status = 'not_requested'
        self.manual_since = None
        self.directory = Path(args.output) / datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        self.directory.mkdir(parents=True)
        (self.directory / 'settings.json').write_text(json.dumps(vars(args), indent=2) + '\n')
        self.csv_file = (self.directory / 'samples.csv').open('w', newline='')
        columns = ['epoch_sec', 'phase', 'trial', 'requested_duty', 'sent_duty',
                   'hardware_output', 'pulse_sec', 'pose_age_sec', 'x', 'y', 'yaw_rad',
                   'motive_stamp_sec', 'vx_mps', 'vy_mps', 'speed_mps', 'travel_m',
                   'after_neutral_travel_m',
                   'reported_throttle_pwm_us',
                   'reported_steering_pwm_us', 'pwm_age_sec', 'command_ack']
        self.writer = csv.DictWriter(self.csv_file, fieldnames=columns)
        self.writer.writeheader()
        self.status_pub = self.create_publisher(String, '/duty_calibration/status', 10)
        if args.state_input_type == 'pose_stamped':
            self.input_topic = args.pose_topic
            self.create_subscription(PoseStamped, args.pose_topic, self.on_pose_stamped, 10)
        else:
            self.input_topic = args.odom_topic
            self.create_subscription(Odometry, args.odom_topic, self.on_pose, 10)
        self.create_subscription(Bool, '/controller/stop', self.on_stop, 10)
        self.create_service(Trigger, '/duty_calibration/start', self.start_trial)
        self.create_service(Trigger, '/duty_calibration/arm', self.arm_vehicle)
        self.create_service(Trigger, '/duty_calibration/stop', self.stop_trial)
        self.create_timer(0.05, self.tick)
        self.get_logger().info(f'Ready in neutral; output={args.drive}, log={self.directory}')
        self.get_logger().info(
            'Each START runs one pulse. Reposition the car before the next START.')
        self.was_startable = False

    def on_pose_stamped(self, message):
        p, q = message.pose.position, message.pose.orientation
        now = time.monotonic()
        stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
        if not all(math.isfinite(v) for v in (p.x, p.y, q.x, q.y, q.z, q.w)):
            self.pose_at = -math.inf
            return
        if self.args.quaternion:
            if abs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1) > 1e-3:
                self.pose_at = -math.inf
                return
            yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        else:
            yaw = q.z
        velocity = self.pose_velocity.update(p.x, p.y, stamp if stamp > 0 else now)
        if velocity is None:
            self.pose_at = -math.inf
            return
        vx, vy = velocity
        self.pose = (p.x, p.y, yaw, math.hypot(vx, vy), vx, vy)
        self.pose_at, self.pose_stamp_sec = now, stamp

    def on_pose(self, message):
        p, q = message.pose.pose.position, message.pose.pose.orientation
        v = message.twist.twist.linear
        values = [p.x, p.y, v.x, v.y, q.x, q.y, q.z, q.w]
        if not all(math.isfinite(value) for value in values):
            self.pose_at = -math.inf
            return
        if self.args.quaternion:
            if abs(sum(value * value for value in (q.x, q.y, q.z, q.w)) - 1) > 1e-3:
                self.pose_at = -math.inf
                return
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        else:
            yaw = q.z
        self.pose = (p.x, p.y, yaw, math.hypot(v.x, v.y), v.x, v.y)
        self.pose_at = time.monotonic()
        self.pose_stamp_sec = message.header.stamp.sec + message.header.stamp.nanosec / 1e9

    def check_graph(self, now):
        if now - self.graph_at >= 0.25:
            names = self.get_node_names()
            self.conflict = bool({'upper_planner_node', 'lower_tracking_mpc_node'} & set(names))
            self.conflict |= names.count('duty_calibration') > 1
            self.graph_at = now

    def ready(self, now):
        return (self.pose is not None and now - self.pose_at <= 0.20 and not self.conflict
                and (not self.link or (self.link.ready(now) and not self.link.rejected)))

    def start_blockers(self, now):
        reasons = []
        if self.blocked_reason:
            reasons.append(f'Previous stop: {self.blocked_reason}; review and restart')
        if self.stopping_at is not None or (self.pulse and self.pulse.active):
            reasons.append('A pulse or stop confirmation is still in progress')
        if self.pose is None:
            reasons.append(f'No valid Motive {self.args.state_input_type}: {self.input_topic}')
        elif now - self.pose_at > .2:
            reasons.append(f'Motive stale/invalid: {self.input_topic}')
        if self.conflict:
            reasons.append('Another controller/calibration node is running')
        if self.link:
            if self.link.peer is None:
                reasons.append('No PX4 heartbeat from MAVProxy on UDP14540')
            elif now - self.link.heartbeat_at > 3:
                reasons.append('PX4 heartbeat is stale')
            if not self.link.armed:
                reasons.append('PX4 is not ARMED')
            if self.link.main_mode != 1:
                reasons.append('PX4 is not MANUAL; use arm to prepare MANUAL then ARM')
            if self.link.rejected:
                reasons.append(f'PX4 rejected actuator command: ACK={self.link.last_ack}')
        if not reasons and (self.stationary_at is None or now - self.stationary_at < 1):
            speed = self.pose[3] if self.pose else None
            reasons.append(f'Waiting for speed <0.03m/s continuously for 1s; speed={speed}')
        if self.index >= len(self.args.duties):
            reasons.append('All pulses complete; restart for repeats')
        return reasons

    def start_trial(self, request, response):
        del request
        now = time.monotonic()
        self.check_graph(now)
        reasons = self.start_blockers(now)
        if reasons:
            response.success, response.message = False, '; '.join(reasons)
            return response
        self.trial += 1
        self.rows = []
        self.stop_distance, self.stop_position = 0.0, None
        self.pulse = Pulse(self.args.duties[self.index], self.args.duration,
                           self.args.max_distance, self.args.max_speed)
        self.pulse.start(now, self.pose)
        response.success = True
        response.message = f'Trial {self.trial}: duty={self.pulse.duty:.3f}, steering=0'
        self.get_logger().info(response.message)
        return response

    def arm_vehicle(self, request, response):
        del request
        now = time.monotonic()
        self.check_graph(now)
        reasons = []
        if not self.args.drive or not self.link:
            reasons.append('ARM requires the experiment to run with --drive')
        elif self.link.peer is None or now - self.link.heartbeat_at > 3:
            reasons.append('Need fresh PX4 heartbeat from MAVProxy')
        elif self.link.rejected:
            reasons.append(f'PX4 actuator output rejected: ACK={self.link.last_ack}')
        if self.pose is None or now - self.pose_at > .2:
            reasons.append(f'Need fresh Motive: {self.input_topic}')
        if self.stationary_at is None or now - self.stationary_at < 1:
            reasons.append('Need speed <0.03m/s continuously for 1s before ARM')
        if self.conflict or self.blocked_reason:
            reasons.append('Controller conflict or a previous stop fault; review and restart')
        if self.stopping_at is not None or (self.pulse and self.pulse.active):
            reasons.append('Cannot ARM during a pulse or stop confirmation')
        if reasons:
            response.success, response.message = False, '; '.join(reasons)
            return response
        if self.link.armed and self.link.main_mode == 1:
            response.success, response.message = True, 'PX4 already ARMED; START is separate.'
            return response
        if self.arm_requested_at is not None:
            response.success, response.message = False, 'ARM pending; check status.'
            return response
        self.link.request_manual_mode()
        self.arm_requested_at, self.arm_status = now, 'waiting_manual'
        self.manual_since = None
        response.success = True
        response.message = ('MANUAL requested with neutral joystick/throttle; '
                            'normal ARM follows confirmed MANUAL. START is separate.')
        return response

    def on_stop(self, message):
        if message.data:
            self.stop_trial(None, Trigger.Response())

    def begin_stop(self, now):
        if self.stopping_at is None:
            self.stopping_at = now
            self.stop_distance = 0.0
            self.stop_position = self.pose[:2] if self.pose else None

    def stop_trial(self, request, response):
        del request
        if self.arm_requested_at is not None:
            self.arm_requested_at, self.arm_status = None, 'cancelled_by_stop'
        if self.pulse and self.pulse.active:
            self.pulse.stop('manual_stop')
            self.begin_stop(time.monotonic())
        if self.link:
            self.link.send(0.0)
        response.success, response.message = True, 'Neutral latched; no automatic restart.'
        return response

    def save_trial(self, stop_status):
        result = dict(trial=self.trial, duty=self.pulse.duty, hardware_output=self.args.drive,
                      reason=self.pulse.reason, stop_status=stop_status,
                      output_duration_sec=(
                          (self.stopping_at if self.stopping_at is not None
                           else time.monotonic()) - self.pulse.started),
                      travel_at_neutral_m=self.pulse.distance,
                      after_neutral_travel_m=self.stop_distance,
                      time_until_stationary_sec=(
                          max(0.0, self.stationary_at - self.stopping_at)
                          if stop_status == 'stationary' and self.stopping_at is not None
                          and self.stationary_at is not None else None), **summarize(self.rows))
        # Never turn a guard-truncated or dry run into a calibrated speed point.
        result['usable_speed_point'] = bool(
            self.args.drive and result['steady_candidate']
            and result['reason'] == 'completed' and stop_status == 'stationary')
        self.results.append(result)
        if self.pulse.reason != 'completed' or stop_status != 'stationary':
            self.blocked_reason = (self.pulse.reason if self.pulse.reason != 'completed'
                                   else stop_status)
        (self.directory / 'trials.json').write_text(json.dumps(self.results, indent=2) + '\n')
        with (self.directory / 'summary.csv').open('w', newline='') as output:
            summary = csv.DictWriter(output, fieldnames=list(result))
            summary.writeheader()
            summary.writerows(self.results)
        self.csv_file.flush()
        self.get_logger().info(json.dumps(result))
        if (self.args.drive and result['reason'] == 'completed'
                and result['tail_samples'] >= 8 and not result['motion_detected']):
            self.get_logger().warn(
                f'No motion detected at duty={self.pulse.duty:.3f}; '
                f'reported throttle PWM={result["tail_reported_throttle_pwm_us"]} us. '
                'Excluded from moving-speed mapping; check before the next manual trial.')
        self.index += 1
        self.stopping_at = None

    def tick(self):
        now = time.monotonic()
        self.check_graph(now)
        if self.link:
            self.link.poll(now)
            if self.arm_requested_at is not None:
                if (self.pose is None or now - self.pose_at > .2 or self.pose[3] >= .03
                        or self.conflict or now - self.link.heartbeat_at > 3):
                    self.arm_status, self.arm_requested_at = 'cancelled_input_not_ready', None
                elif self.arm_status == 'waiting_manual':
                    if self.link.main_mode == 1:
                        self.manual_since = self.manual_since or now
                    else:
                        self.manual_since = None
                    if self.manual_since is not None and now - self.manual_since >= 1.0:
                        if self.link.armed:
                            self.arm_status, self.arm_requested_at = 'confirmed', None
                        else:
                            self.link.request_arm()
                            self.arm_status, self.arm_requested_at = 'requested', now
                            self.get_logger().info(
                                'MANUAL confirmed; normal ARM requested at throttle 0.')
                    elif now - self.arm_requested_at > 5:
                        self.arm_status, self.arm_requested_at = 'manual_timeout', None
                        self.get_logger().warn('MANUAL was not confirmed; ARM was not requested.')
                elif self.link.armed and self.link.heartbeat_at >= self.arm_requested_at:
                    self.arm_status, self.arm_requested_at = 'confirmed', None
                    self.get_logger().info(
                        'PX4 ARM confirmed; throttle remains 0. START is separate.')
                elif self.link.arm_ack not in (None, mav.MAV_RESULT_ACCEPTED,
                                               mav.MAV_RESULT_IN_PROGRESS):
                    self.arm_status, self.arm_requested_at = 'rejected', None
                    self.get_logger().warn(
                        f'PX4 rejected ARM: ACK={self.link.arm_ack} {self.link.last_statustext}')
                elif now - self.arm_requested_at > 5:
                    self.arm_status, self.arm_requested_at = 'timeout', None
                    self.get_logger().warn(
                        'ARM heartbeat confirmation timed out; throttle remains 0.')
        good = self.ready(now)
        # Stationary Motive must be established before ARM, independently of armed state.
        stationary = (self.pose is not None and now - self.pose_at <= .2
                      and self.pose[3] < .03 and not self.conflict)
        self.stationary_at = (self.stationary_at or now) if stationary else None
        was_active = self.pulse is not None and self.pulse.active
        duty = (self.pulse.output(now, self.pose, self.pose_at, good, self.conflict)
                if self.pulse else 0.0)
        if was_active and not self.pulse.active:
            self.begin_stop(now)
            self.get_logger().info(f'Neutral: {self.pulse.reason}; waiting for stationary Motive.')
        if self.link:
            self.link.send(duty)
        if self.stopping_at is not None and self.pose and now - self.pose_at <= 0.20:
            if self.stop_position is not None:
                self.stop_distance += math.dist(self.stop_position, self.pose[:2])
            self.stop_position = self.pose[:2]
        phase = 'pulse' if self.pulse and self.pulse.active else (
            'stopping' if self.stopping_at is not None else 'idle')
        row = dict(epoch_sec=time.time(), phase=phase, trial=self.trial,
                   requested_duty=self.pulse.duty if self.pulse else 0.0,
                   sent_duty=duty if self.link else 0.0, hardware_output=self.args.drive,
                   pulse_sec=now - self.pulse.started if self.pulse else 0.0,
                   pose_age_sec=now - self.pose_at, x=self.pose[0] if self.pose else '',
                   y=self.pose[1] if self.pose else '', yaw_rad=self.pose[2] if self.pose else '',
                   motive_stamp_sec=self.pose_stamp_sec if self.pose else '',
                   vx_mps=self.pose[4] if self.pose else '',
                   vy_mps=self.pose[5] if self.pose else '',
                   speed_mps=self.pose[3] if self.pose else '',
                   travel_m=self.pulse.distance if self.pulse else 0.0,
                   after_neutral_travel_m=self.stop_distance,
                   reported_throttle_pwm_us='', reported_steering_pwm_us='',
                   pwm_age_sec='', command_ack='')
        if self.link:
            age = now - self.link.pwm_at
            row['pwm_age_sec'] = age if math.isfinite(age) else ''
            if age <= 0.25:
                row['reported_throttle_pwm_us'] = self.link.pwm.get(9, 0) or ''
                row['reported_steering_pwm_us'] = self.link.pwm.get(10, 0) or ''
            row['command_ack'] = self.link.last_ack if self.link.last_ack is not None else ''
        self.writer.writerow(row)
        if was_active:
            self.rows.append(row)
        if self.stopping_at is not None:
            if self.stationary_at is not None and now - self.stationary_at >= 1.0:
                self.save_trial('stationary')
            elif now - self.stopping_at >= 5.0:
                self.save_trial('not_confirmed')
        can_start = (good and self.stationary_at is not None
                     and now - self.stationary_at >= 1.0 and phase == 'idle'
                     and self.index < len(self.args.duties) and not self.blocked_reason)
        if can_start and not self.was_startable:
            self.get_logger().info(
                f'Stationary and ready. Next duty={self.args.duties[self.index]:.3f}; '
                'START required.')
        self.was_startable = can_start
        self.status_pub.publish(String(data=json.dumps(dict(
            phase=phase, ready=good,
            next_duty=(self.args.duties[self.index]
                       if self.index < len(self.args.duties) else None),
            speed_mps=self.pose[3] if self.pose else None, trial=self.trial,
            can_start=can_start,
            motive_age_sec=(now - self.pose_at if math.isfinite(self.pose_at) else None),
            armed=self.link.armed if self.link else None, controller_conflict=self.conflict,
            arm_status=self.arm_status, arm_ack=self.link.arm_ack if self.link else None,
            px4_main_mode=self.link.main_mode if self.link else None,
            px4_statustext=self.link.last_statustext if self.link else '',
            blocked_reason=self.blocked_reason,
            input_topic=self.input_topic, input_type=self.args.state_input_type,
            speed_source=('motive_position_difference_0.1s'
                          if self.args.state_input_type == 'pose_stamped' else 'motive_twist'),
            start_blockers=self.start_blockers(now),
            last_reason=self.pulse.reason if self.pulse else '',
            hardware_output=self.args.drive))))

    def close(self):
        # Neutral first; result-file writes must not delay the final output.
        if self.link:
            self.link.send(0.0)
        if self.pulse and self.pulse.active:
            self.pulse.stop('process_exit')
            self.save_trial('not_confirmed')
        elif self.stopping_at is not None:
            self.save_trial('not_confirmed')
        if self.link:
            for _ in range(5):
                self.link.send(0.0)
                time.sleep(0.02)
            self.link.close()
        self.csv_file.close()


def parse_args(argv=None):
    workspace = Path(__file__).resolve().parents[3]
    profile_path = workspace / 'src/virtual_control/config/model_real_control.yaml'
    profile = yaml.safe_load(profile_path.read_text())
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--drive', action='store_true',
                        help='Enable MAVLink output; START still required.')
    parser.add_argument('--duties', nargs='+', type=float, default=[0.03, 0.05, 0.07])
    parser.add_argument('--duration', type=float, default=1.0,
                        help='Maximum pulse duration in seconds (up to 15).')
    distance_options = parser.add_mutually_exclusive_group()
    distance_options.add_argument('--max-distance', type=float, default=0.4)
    distance_options.add_argument(
        '--no-distance-limit', dest='max_distance', action='store_const', const=None,
        help='Disable distance cutoff; keep time and input stop guards.')
    speed_options = parser.add_mutually_exclusive_group()
    speed_options.add_argument('--max-speed', type=float, default=0.5)
    speed_options.add_argument(
        '--no-speed-limit', dest='max_speed', action='store_const', const=None,
        help='Disable speed cutoff; keep all other stop guards.')
    parser.add_argument('--port', type=int, default=14540)
    odom_topic = profile['lower_tracking_mpc_node']['ros__parameters']['odom_topic']
    lower = profile['lower_tracking_mpc_node']['ros__parameters']
    parser.add_argument('--state-input-type', choices=['odom', 'pose_stamped'],
                        default=lower.get('state_input_type', 'odom'))
    parser.add_argument('--odom-topic', default=odom_topic)
    parser.add_argument('--pose-topic', default=lower.get(
        'pose_stamped_topic', '/motive/vehicle/pose'))
    parser.add_argument('--quaternion', action='store_true')
    parser.add_argument('--output', default=str(workspace / 'duty_calibration_runs'))
    args = parser.parse_args(argv)
    if (any(not math.isfinite(v) or not 0 < v <= 0.15 for v in args.duties)
            or not math.isfinite(args.duration) or not 0 < args.duration <= 15
            or (args.max_distance is not None and (
                not math.isfinite(args.max_distance) or not 0 < args.max_distance <= 0.4))
            or (args.max_speed is not None and (
                not math.isfinite(args.max_speed) or not 0 < args.max_speed <= 0.5))
            or not 1 <= args.port <= 65535):
        parser.error('duties (0,.15], duration (0,15], '
                     'distance (0,.4] or --no-distance-limit, '
                     'speed (0,.5] or --no-speed-limit, '
                     'valid UDP port required')
    return args


def main(argv=None):
    args = parse_args(argv)
    rclpy.init(args=[])
    node = None
    try:
        node = Calibration(args)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
