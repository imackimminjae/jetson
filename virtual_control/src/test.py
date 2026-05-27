#!/usr/bin/env python3
import argparse
import math
import time
from pymavlink import mavutil


MAV_CMD_DO_SET_ACTUATOR = 187
MAV_CMD_COMPONENT_ARM_DISARM = 400
PX4_CUSTOM_MAIN_MODE_MANUAL = 1


def px4_custom_mode(main_mode: int, sub_mode: int = 0) -> int:
    return (sub_mode << 24) | (main_mode << 16)


def connect(master: str, baud: int):
    print(f"[MAVLINK] connecting: master={master}, baud={baud}")
    mav = mavutil.mavlink_connection(
        master,
        baud=baud,
        source_system=245,
        source_component=190,
        autoreconnect=True,
    )

    print("[MAVLINK] waiting heartbeat...")
    hb = mav.wait_heartbeat(timeout=10)
    if hb is None:
        raise RuntimeError("Heartbeat timeout. Check MAVProxy/Pixhawk connection.")

    # Match C++ behavior:
    # C++ sets target_system from heartbeat sysid,
    # but forces target_component to 1.
    mav.target_system = mav.target_system
    mav.target_component = 1

    print(
        f"[MAVLINK] heartbeat OK. using target_system={mav.target_system}, "
        f"target_component={mav.target_component}, source=245:190"
    )
    return mav


def poll_mavlink(mav, print_all=False):
    while True:
        msg = mav.recv_match(blocking=False)
        if msg is None:
            break

        msg_type = msg.get_type()

        if msg_type == "COMMAND_ACK":
            print(
                f"[ACK] command={msg.command} result={msg.result} "
                f"progress={getattr(msg, 'progress', 0)} "
                f"result_param2={getattr(msg, 'result_param2', 0)}"
            )

        elif msg_type == "STATUSTEXT":
            try:
                text = msg.text
            except Exception:
                text = str(msg)
            print(f"[STATUSTEXT] severity={msg.severity} text={text}")

        elif msg_type == "HEARTBEAT" and print_all:
            print(f"[HEARTBEAT] {msg}")

        elif print_all:
            print(f"[MAVLINK] {msg_type}: {msg}")


def send_manual_neutral(mav):
    # Same role as C++ mavlinkSendManualNeutral()
    mav.mav.manual_control_send(
        mav.target_system,
        0,  # x
        0,  # y
        0,  # z
        0,  # r
        0,  # buttons
    )


def send_manual_mode(mav):
    custom_mode = px4_custom_mode(PX4_CUSTOM_MAIN_MODE_MANUAL)

    mav.mav.set_mode_send(
        mav.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        custom_mode,
    )

    print(f"[MAVLINK] SET_MODE MANUAL sent: custom_mode={custom_mode}")


def send_arm(mav, arm: bool, force: bool):
    force_code = 21196.0 if force else 0.0

    mav.mav.command_long_send(
        mav.target_system,
        mav.target_component,
        MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1.0 if arm else 0.0,
        force_code,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    )

    print(f"[MAVLINK] {'ARM' if arm else 'DISARM'} sent: force={force}")


def send_actuator(mav, throttle_norm: float, steering_norm: float):
    # Same as C++ mavlinkSendActuator()
    throttle_norm = max(-1.0, min(1.0, float(throttle_norm)))
    steering_norm = max(-1.0, min(1.0, float(steering_norm)))

    nan = float("nan")

    mav.mav.command_long_send(
        mav.target_system,
        mav.target_component,
        MAV_CMD_DO_SET_ACTUATOR,
        0,
        throttle_norm,   # param1: throttle
        steering_norm,   # param2: steering
        nan,
        nan,
        nan,
        nan,
        0.0,
    )


def run_cpp_like_hold(
    mav,
    throttle: float,
    steer: float,
    duration: float,
    rate_hz: float,
    auto_manual: bool,
    auto_arm: bool,
    force_arm: bool,
):
    dt = 1.0 / max(rate_hz, 1.0)
    t0 = time.time()
    n = 0

    last_mode_req = 0.0
    last_arm_req = 0.0

    while time.time() - t0 < duration:
        now = time.time()

        # C++ timer order:
        # mavlinkPoll()
        poll_mavlink(mav)

        # mavlinkSendManualNeutral()
        send_manual_neutral(mav)

        # mavlinkManageModeAndArm()
        if auto_manual and now - last_mode_req > 1.0:
            send_manual_mode(mav)
            last_mode_req = now

        if auto_arm and now - last_arm_req > 1.0:
            send_arm(mav, True, force_arm)
            last_arm_req = now

        # mavlinkSendHeldActuator()
        send_actuator(mav, throttle, steer)

        n += 1
        time.sleep(dt)

    poll_mavlink(mav)
    print(
        f"[MAVLINK] hold done: throttle={throttle:.3f}, steer={steer:.3f}, "
        f"duration={duration:.2f}s, sent={n}"
    )


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--master",
        default="udpin:0.0.0.0:14540",
        help="Use with MAVProxy --out=127.0.0.1:14540",
    )
    parser.add_argument("--baud", type=int, default=115200)

    parser.add_argument(
        "--test",
        choices=["neutral", "straight", "left", "right"],
        default="neutral",
    )
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--rate", type=float, default=20.0)

    parser.add_argument("--throttle", type=float, default=0.15)
    parser.add_argument("--steer", type=float, default=0.2)

    parser.add_argument("--set-manual", action="store_true")
    parser.add_argument("--arm", action="store_true")
    parser.add_argument("--force-arm", action="store_true")
    parser.add_argument("--disarm-after", action="store_true")

    args = parser.parse_args()

    mav = connect(args.master, args.baud)

    if args.test == "neutral":
        throttle = 0.0
        steer = 0.0
    elif args.test == "straight":
        throttle = args.throttle
        steer = 0.0
    elif args.test == "left":
        throttle = args.throttle
        steer = -abs(args.steer)
    elif args.test == "right":
        throttle = args.throttle
        steer = abs(args.steer)

    print(
        f"[TEST] {args.test}: throttle={throttle:.3f}, steer={steer:.3f}, "
        f"duration={args.duration:.2f}s"
    )

    run_cpp_like_hold(
        mav,
        throttle=throttle,
        steer=steer,
        duration=args.duration,
        rate_hz=args.rate,
        auto_manual=args.set_manual,
        auto_arm=args.arm,
        force_arm=args.force_arm,
    )

    print("[TEST] final neutral")
    run_cpp_like_hold(
        mav,
        throttle=0.0,
        steer=0.0,
        duration=0.5,
        rate_hz=args.rate,
        auto_manual=False,
        auto_arm=False,
        force_arm=False,
    )

    if args.disarm_after:
        send_arm(mav, False, False)
        time.sleep(0.2)
        poll_mavlink(mav)


if __name__ == "__main__":
    main()