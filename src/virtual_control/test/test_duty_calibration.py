"""Bounded manual pulses, wire mapping and measurement-quality checks; no hardware."""
import importlib.util
import math
import json
import csv
import os
from pathlib import Path
import sys
import time

import pytest

SCRIPTS = Path(__file__).resolve().parents[1] / 'scripts'
SPEC = importlib.util.spec_from_file_location('duty_calibration', SCRIPTS / 'duty_calibration.py')
CAL = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = CAL
SPEC.loader.exec_module(CAL)


def pulse():
    result = CAL.Pulse(0.05, 1.0, 0.4, 0.5)
    result.start(10.0, (0.0, 0.0, 0.0, 0.0))
    return result


def test_wire_command_matches_lower_actuator_sets():
    message = CAL.actuator_message(0.05)
    assert message.command == 187
    assert (message.target_system, message.target_component) == (1, 1)
    assert message.param1 == 0.05 and message.param2 == 0
    assert all(math.isnan(value) for value in (
        message.param3, message.param4, message.param5, message.param6))
    assert message.param7 == 0
    # Decode the actual MAVLink 2 wire packet, including ignored actuator NaNs.
    encoder = CAL.mav.MAVLink(None, srcSystem=245, srcComponent=190)
    decoded = CAL.mav.MAVLink(None).parse_buffer(message.pack(encoder))[0]
    assert decoded.command == 187 and decoded.param1 == pytest.approx(0.05)
    assert decoded.param2 == 0 and math.isnan(decoded.param3)


def test_fake_mavproxy_heartbeat_wire_output_and_rejection(monkeypatch):
    class FakeSocket:
        def __init__(self):
            self.incoming, self.sent = [], []

        def bind(self, address):
            self.bound = address

        def setblocking(self, value):
            pass

        def recvfrom(self, size):
            if self.incoming:
                return self.incoming.pop(0), ('127.0.0.1', 12345)
            raise BlockingIOError

        def sendto(self, data, peer):
            self.sent.append(data)
            return len(data)

    sink = FakeSocket()
    monkeypatch.setattr(CAL.socket, 'socket', lambda *args: sink)
    link = CAL.Link(14540)
    assert sink.bound == ('0.0.0.0', 14540)
    assert not link.ready(1.)
    px4 = CAL.mav.MAVLink(None, srcSystem=1, srcComponent=1)
    heartbeat = CAL.mav.MAVLink_heartbeat_message(
        CAL.mav.MAV_TYPE_GROUND_ROVER, CAL.mav.MAV_AUTOPILOT_PX4,
        CAL.mav.MAV_MODE_FLAG_SAFETY_ARMED, 1 << 16, CAL.mav.MAV_STATE_ACTIVE, 3)
    sink.incoming.append(heartbeat.pack(px4))
    link.poll(10.)
    assert link.ready(10.)
    assert not link.ready(13.1)
    link.send(.05)
    link.send(0.)
    decoded = [CAL.mav.MAVLink(None).parse_buffer(packet)[0] for packet in sink.sent]
    commands = [message for message in decoded if message.get_type() == 'COMMAND_LONG']
    assert [message.command for message in commands] == [511, 187, 187]
    assert commands[1].param1 == pytest.approx(.05) and commands[1].param2 == 0
    assert commands[2].param1 == 0
    sticks = [message for message in decoded if message.get_type() == 'MANUAL_CONTROL']
    assert len(sticks) == 2
    assert all((message.x, message.y, message.z, message.r) == (0, 0, 0, 0)
               for message in sticks)
    rejection = CAL.mav.MAVLink_command_ack_message(187, CAL.mav.MAV_RESULT_UNSUPPORTED)
    sink.incoming.append(rejection.pack(px4))
    link.poll(10.1)
    assert link.rejected
    link.armed = False
    assert not link.ready(10.1)
    link.request_arm()
    arm = CAL.mav.MAVLink(None).parse_buffer(sink.sent[-1])[0]
    neutral = CAL.mav.MAVLink(None).parse_buffer(sink.sent[-2])[0]
    assert neutral.command == 187 and neutral.param1 == 0
    assert arm.command == 400 and arm.param1 == 1 and arm.param2 == 0
    assert not link.armed  # A sent ARM command is not an armed heartbeat.
    link.request_manual_mode()
    mode = CAL.mav.MAVLink(None).parse_buffer(sink.sent[-1])[0]
    assert mode.get_type() == 'SET_MODE' and mode.custom_mode == 1 << 16
    text = CAL.mav.MAVLink_statustext_message(
        2, b'Arming denied: Resolve system health failures firs', 0, 0)
    tail = CAL.mav.MAVLink_statustext_message(2, b't', 0, 1)
    sink.incoming.extend([text.pack(px4), tail.pack(px4)])
    link.poll(10.2)
    assert link.last_statustext == 'Arming denied: Resolve system health failures first'


def test_arming_fake_link_never_starts_pulse_or_bypasses_heartbeat(tmp_path, monkeypatch):
    if os.environ.get('DUTY_CALIBRATION_ROS_TEST') != '1':
        pytest.skip('isolated ROS opt-in required')
    assert os.environ.get('ROS_DOMAIN_ID') == '179'
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from std_srvs.srv import Trigger

    class FakeLink:
        def __init__(self):
            self.peer, self.armed, self.rejected = True, False, False
            self.main_mode, self.mode_requests = 4, 0
            self.heartbeat_at, self.pwm_at = time.monotonic(), -math.inf
            self.pwm, self.last_ack, self.arm_ack = {}, None, None
            self.last_statustext = ''
            self.commands, self.arm_requests = [], 0

        def poll(self, now):
            self.heartbeat_at = now

        def ready(self, now):
            return self.armed and self.main_mode == 1 and now - self.heartbeat_at <= 3

        def request_arm(self):
            self.arm_requests += 1
            self.commands.append(0.)
            self.arm_ack = CAL.mav.MAV_RESULT_ACCEPTED

        def request_manual_mode(self):
            self.mode_requests += 1
            self.commands.append(0.)

        def send(self, duty):
            self.commands.append(duty)

        def close(self):
            pass

    fake = FakeLink()
    monkeypatch.setattr(CAL, 'Link', lambda port: fake)
    args = CAL.parse_args(['--drive', '--output', str(tmp_path)])
    rclpy.init()
    node = probe = None
    try:
        node = CAL.Calibration(args)
        probe = rclpy.create_node('fake_px4_arm_probe')
        pub = probe.create_publisher(PoseStamped, args.pose_topic, 10)
        client = probe.create_client(Trigger, '/duty_calibration/arm')

        def spin(duration):
            end = time.monotonic() + duration
            while time.monotonic() < end:
                pose = PoseStamped()
                pose.header.stamp = probe.get_clock().now().to_msg()
                pub.publish(pose)
                rclpy.spin_once(node, timeout_sec=.005)
                rclpy.spin_once(probe, timeout_sec=.005)
                time.sleep(.01)

        response = node.arm_vehicle(None, Trigger.Response())
        assert not response.success and fake.arm_requests == 0  # No Motive, no ARM.
        spin(1.8)
        fake.rejected, fake.last_ack = True, CAL.mav.MAV_RESULT_UNSUPPORTED
        assert not node.arm_vehicle(None, Trigger.Response()).success
        assert fake.arm_requests == 0
        fake.rejected, fake.last_ack = False, CAL.mav.MAV_RESULT_ACCEPTED
        future = client.call_async(Trigger.Request())
        spin(.3)
        assert future.done() and future.result().success
        assert fake.arm_requests == 0 and fake.mode_requests == 1 and node.pulse is None
        assert not node.start_trial(None, Trigger.Response()).success
        assert all(duty == 0 for duty in fake.commands)
        fake.main_mode = 1
        spin(1.2)
        assert fake.arm_requests == 1 and not fake.armed
        assert node.pulse is None and all(duty == 0 for duty in fake.commands)
        fake.armed = True
        spin(.2)
        assert node.arm_status == 'confirmed'
        assert node.pulse is None and all(duty == 0 for duty in fake.commands)
        assert node.start_trial(None, Trigger.Response()).success
        spin(.15)
        assert any(duty == .03 for duty in fake.commands)
        node.stop_trial(None, Trigger.Response())
        assert fake.commands[-1] == 0
    finally:
        if node:
            node.close()
            node.destroy_node()
        if probe:
            probe.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize('now,pose,age,ready,conflict,reason', [
    (10.3, (0., 0., 0., 0.), 10., True, False, 'motive_lost'),
    (10.1, (0., 0., 0., .5), 10.1, True, False, 'speed_limit'),
    (10.1, (.16, 0., 0., .1), 10.1, True, False, 'pose_jump'),
    (10.1, (0., .11, 0., .1), 10.1, True, False, 'straight_line_deviation'),
    (10.1, (0., 0., math.radians(11), .1), 10.1, True, False, 'straight_line_deviation'),
    (10.1, (0., 0., 0., .1), 10.1, False, False, 'heartbeat_arm_or_command_fault'),
    (10.1, (0., 0., 0., .1), 10.1, False, True, 'controller_conflict'),
    (11., (0., 0., 0., 0.), 11., True, False, 'completed'),
])
def test_faults_latch_neutral_without_automatic_resume(now, pose, age, ready, conflict, reason):
    trial = pulse()
    assert trial.output(now, pose, age, ready, conflict) == 0
    assert trial.reason == reason
    assert trial.output(now + .01, (0., 0., 0., 0.), now + .01, True) == 0


def test_distance_bound_counts_travel_even_after_returning_toward_origin():
    trial = pulse()
    for index, x in enumerate((.1, .2, .3, .2)):
        now = 10.05 + .05 * index
        result = trial.output(now, (x, 0., 0., .2), now, True)
    assert result == 0
    assert trial.reason == 'distance_limit'


@pytest.mark.parametrize('argv', [
    ['--duties', 'nan'], ['--duties', '.16'], ['--duties', '-.01'],
    ['--duration', 'nan'], ['--duration', '15.01'], ['--max-distance', '.6'],
    ['--max-speed', '1'], ['--port', '0'],
])
def test_invalid_or_excessive_pulses_rejected_before_connecting(argv):
    with pytest.raises(SystemExit):
        CAL.parse_args(argv)


def test_unsettled_speed_does_not_become_a_steady_mapping_point():
    rows = [dict(phase='pulse', sent_duty=.05, pulse_sec=i / 20, speed_mps=i / 40)
            for i in range(1, 20)]
    assert not CAL.summarize(rows)['steady_candidate']
    for row in rows:
        row['speed_mps'] = .2
    assert CAL.summarize(rows)['steady_candidate']
    assert CAL.summarize(rows)['tail_reported_throttle_pwm_us'] is None


def test_optional_speed_cutoff_keeps_duration_distance_and_input_stops():
    args = CAL.parse_args(['--no-speed-limit', '--duration', '15'])
    assert args.max_speed is None and args.duration == 15
    assert CAL.parse_args([]).max_speed == .5
    with pytest.raises(SystemExit):
        CAL.parse_args(['--no-speed-limit', '--max-speed', '.5'])
    for reason in ('completed', 'distance_limit', 'motive_lost', 'manual_stop'):
        trial = CAL.Pulse(.1, args.duration, args.max_distance, args.max_speed)
        trial.start(10., (0., 0., 0., 0.))
        assert trial.output(10.05, (.04, 0., 0., .8), 10.05, True) == .1
        if reason == 'completed':
            assert trial.output(24.99, (.04, 0., 0., .8), 24.99, True) == .1
            output = trial.output(25., (.04, 0., 0., .8), 25., True)
        elif reason == 'distance_limit':
            for i, x in enumerate((.14, .24, .34, .4)):
                now = 10.1 + .05*i
                output = trial.output(now, (x, 0., 0., .8), now, True)
        elif reason == 'motive_lost':
            output = trial.output(10.3, (.04, 0., 0., .8), 10.05, True)
        else:
            output = trial.stop('manual_stop')
        assert output == 0 and trial.reason == reason
        assert trial.output(25.1, (.04, 0., 0., .8), 25.1, True) == 0


def test_long_pulse_reports_average_peak_and_final_second_separately():
    rows = [dict(phase='pulse', sent_duty=.1, pulse_sec=i / 20,
                 speed_mps=.2 if i < 40 else .6) for i in range(1, 61)]
    rows.append(dict(phase='stopping', sent_duty=0., pulse_sec=3.05, speed_mps=1.))
    result = CAL.summarize(rows)
    assert result['mean_speed_mps'] == pytest.approx((39*.2 + 21*.6)/60)
    assert result['peak_speed_mps'] == .6
    assert result['tail_median_speed_mps'] == .6
    assert result['tail_samples'] == 21
    assert result['steady_candidate']


@pytest.mark.parametrize('reason', [
    'completed', 'motive_lost', 'heartbeat_arm_or_command_fault',
    'controller_conflict', 'straight_line_deviation', 'manual_stop'])
def test_distance_and_speed_can_be_disabled_with_other_stops_retained(reason):
    args = CAL.parse_args(['--no-distance-limit', '--no-speed-limit', '--duration', '15'])
    assert args.max_distance is None and args.max_speed is None
    assert CAL.parse_args([]).max_distance == .4
    with pytest.raises(SystemExit):
        CAL.parse_args(['--no-distance-limit', '--max-distance', '.4'])
    trial = CAL.Pulse(.1, args.duration, args.max_distance, args.max_speed)
    trial.start(10., (0., 0., 0., 0.))
    for i in range(1, 13):
        now = 10. + .05*i
        assert trial.output(now, (.1*i, 0., 0., 2.), now, True) == .1
    assert trial.distance > 1.
    pose = (1.2, 0., 0., 2.)
    if reason == 'completed':
        assert trial.output(24.99, pose, 24.99, True) == .1
        output = trial.output(25., pose, 25., True)
    elif reason == 'motive_lost':
        output = trial.output(10.9, pose, 10.6, True)
    elif reason == 'heartbeat_arm_or_command_fault':
        output = trial.output(10.7, pose, 10.7, False)
    elif reason == 'controller_conflict':
        output = trial.output(10.7, pose, 10.7, True, True)
    elif reason == 'straight_line_deviation':
        output = trial.output(10.7, (1.2, .11, 0., 2.), 10.7, True)
    else:
        output = trial.stop('manual_stop')
    assert output == 0 and trial.reason == reason
    assert trial.output(25.1, pose, 25.1, True) == 0


def test_dry_run_never_claims_a_calibrated_output_point():
    rows = [dict(phase='pulse', sent_duty=0., pulse_sec=i / 20, speed_mps=.2)
            for i in range(1, 20)]
    assert CAL.summarize(rows)['samples'] == 0
    assert not CAL.summarize(rows)['steady_candidate']


@pytest.mark.parametrize('speed,moving', [
    (0., False), (.00018, False), (.029, False), (.03, True), (.2, True)])
def test_stationary_noise_is_excluded_from_moving_speed_mapping(speed, moving):
    rows = [dict(phase='pulse', sent_duty=.03, pulse_sec=i / 20, speed_mps=speed,
                 reported_throttle_pwm_us=1515) for i in range(1, 21)]
    result = CAL.summarize(rows)
    assert result['motion_detected'] is moving
    assert result['steady_candidate'] is moving
    assert result['tail_reported_throttle_pwm_us'] == 1515


def test_pose_velocity_window_uses_physical_metres_and_rejects_reordering_and_gaps():
    estimator = CAL.PoseVelocity()
    assert estimator.update(1., 2., 10.) is None
    result = None
    for i in range(1, 13):
        result = estimator.update(1. + .002*i, 2., 10. + .01*i)
    assert result == pytest.approx((.2, 0.))
    assert estimator.update(50., 50., 10.1) is None
    assert estimator.update(1.026, 2., 10.13) == pytest.approx((.2, 0.))
    assert estimator.update(1.03, 2., 11.) is None  # Input gap resets velocity history.


@pytest.mark.parametrize('stop_cause', ['motive_lost', 'manual_stop'])
@pytest.mark.parametrize('input_type', ['odom', 'pose_stamped'])
def test_external_motive_manual_start_and_loss_with_no_hardware(tmp_path, stop_cause, input_type):
    if os.environ.get('DUTY_CALIBRATION_ROS_TEST') != '1':
        pytest.skip('isolated ROS opt-in required')
    assert os.environ.get('ROS_DOMAIN_ID') == '179'
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    import rclpy
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import PoseStamped
    from std_srvs.srv import Trigger
    from std_msgs.msg import Bool
    args = CAL.parse_args(['--output', str(tmp_path), '--state-input-type', input_type])
    rclpy.init()
    node = probe = None
    try:
        node = CAL.Calibration(args)
        probe = rclpy.create_node('synthetic_calibration_probe')
        msg_type = PoseStamped if input_type == 'pose_stamped' else Odometry
        publisher = probe.create_publisher(msg_type, node.input_topic, 10)
        stop_publisher = probe.create_publisher(Bool, '/controller/stop', 10)
        client = probe.create_client(Trigger, '/duty_calibration/start')
        message = msg_type()
        message.header.frame_id = 'map'
        position = (message.pose.position if input_type == 'pose_stamped'
                    else message.pose.pose.position)

        def spin_for(duration, publish=True, simulate_stop=False):
            started = time.monotonic()
            while time.monotonic() - started < duration:
                if publish:
                    if simulate_stop:
                        moving = node.pulse.active or (
                            node.stopping_at is not None
                            and time.monotonic() - node.stopping_at < .3)
                        if input_type == 'odom':
                            message.twist.twist.linear.x = .1 if moving else 0.
                        if moving:
                            position.x += .001
                    message.header.stamp = probe.get_clock().now().to_msg()
                    publisher.publish(message)
                rclpy.spin_once(node, timeout_sec=.005)
                rclpy.spin_once(probe, timeout_sec=.005)
                time.sleep(.01)

        def start():
            future = client.call_async(Trigger.Request())
            deadline = time.monotonic() + 1
            while not future.done() and time.monotonic() < deadline:
                spin_for(.02)
            assert future.done()
            assert future.result().success, future.result().message

        spin_for(1.8)
        assert node.pulse is None  # Fresh Motive alone never starts output.
        assert node.link is None
        start()
        spin_for(2.7, simulate_stop=True)
        assert node.index == 1
        assert node.results[0]['reason'] == 'completed'
        assert not node.results[0]['usable_speed_point']
        assert node.results[0]['after_neutral_travel_m'] > 0
        assert node.results[0]['time_until_stationary_sec'] >= .25
        spin_for(.2)
        assert node.trial == 1  # No automatic next duty.
        start()
        if stop_cause == 'motive_lost':
            spin_for(.4, publish=False)
        else:
            stop_publisher.publish(Bool(data=True))
            spin_for(.2)
        assert node.pulse.reason == stop_cause
        spin_for(1.3)
        assert not node.pulse.active
        assert node.trial == 2
        assert node.blocked_reason == stop_cause
        response = node.start_trial(Trigger.Request(), Trigger.Response())
        assert not response.success  # Recovered Motive cannot advance to a higher duty.
        node.csv_file.flush()
        rows = list(csv.DictReader((node.directory / 'samples.csv').open()))
        assert rows and all(float(row['sent_duty']) == 0 for row in rows)
        assert all(not result['hardware_output'] for result in
                   json.loads((node.directory / 'trials.json').read_text()))
    finally:
        if node:
            node.close()
            node.destroy_node()
        if probe:
            probe.destroy_node()
        rclpy.shutdown()
