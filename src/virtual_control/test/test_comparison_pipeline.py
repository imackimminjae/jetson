"""Headless ROS pipeline smoke test; hardware transport disabled, isolated domain."""
import os
from pathlib import Path
import subprocess
import time

import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path as PathMsg
from std_msgs.msg import Float64MultiArray


@pytest.mark.parametrize("planner,source", [("nvp", "sparse"), ("mpcc", "sparse"), ("mpcc", "exact")])
def test_planner_feeds_existing_lower_mpc(planner, source, tmp_path):
    upper_binary = os.environ[f"COMPARISON_{planner.upper()}_EXECUTABLE"]
    lower_binary = os.environ["LOWER_TRACKER_EXECUTABLE"]
    split_config = Path(__file__).resolve().parents[1] / "config" / "tracking_control_split.yaml"
    upper = [upper_binary, "--ros-args", "-p", f"planner:={planner}",
             "-p", f"reference_source:={source}"]
    lower = [lower_binary, "--ros-args", "--params-file", str(split_config),
             "-p", "mavlink_enable:=false", "-p", "pixhawk_output_backend:=disabled",
             "-p", "mavlink_auto_arm:=false", "-p", "batch_supervision:=false"]
    processes = []
    rclpy.init()
    node = rclpy.create_node("comparison_pipeline_test_peer")
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL)
    odom_pub = node.create_publisher(Odometry, "/motive/vehicle/odom_map", 10)
    grid_pub = node.create_publisher(OccupancyGrid, "/bev/occupancy_grid", 10)
    path_pub = node.create_publisher(PathMsg, "/navigation/exact_reference_path"
                                     if source == "exact" else "/navigation/global_path", latched)
    traces = []
    node.create_subscription(Float64MultiArray, "/debug/lower_mpc_trace",
                             lambda msg: traces.append(list(msg.data)), 10)
    path = PathMsg()
    path.header.frame_id = "map"
    for x in [0.0, 25.0, 50.0]:
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = x
        pose.pose.orientation.w = 1.0
        path.poses.append(pose)
    bev = OccupancyGrid()
    bev.header.frame_id = "Vehicle"
    bev.info.resolution = 0.3125
    bev.info.width = bev.info.height = 128
    bev.info.origin.position.y = -20.0
    bev.info.origin.orientation.w = 1.0
    bev.data = [0] * (128 * 128)
    odom = Odometry()
    odom.header.frame_id = "map"
    odom.twist.twist.linear.x = 6.0
    with (tmp_path / "nodes.log").open("w+") as log:
        try:
            for command in [upper, lower]:
                processes.append(subprocess.Popen(command, stdout=log, stderr=log))
            deadline = time.monotonic() + 8
            while time.monotonic() < deadline:
                odom.header.stamp = node.get_clock().now().to_msg()
                bev.header.stamp = odom.header.stamp
                odom_pub.publish(odom)
                grid_pub.publish(bev)
                path_pub.publish(path)
                rclpy.spin_once(node, timeout_sec=0.02)
                if any(len(trace) > 10 and trace[10] == 1.0 for trace in traces):
                    break
                assert all(p.poll() is None for p in processes), "node exited early"
            assert any(len(trace) > 10 and trace[10] == 1.0 for trace in traces), (
                f"{planner}/{source}: lower QP never accepted the generated path")
            # Known occupied BEV invalidates the path; existing lower must enter neutral mode.
            bev.data = [100] * (128 * 128)
            traces.clear()
            deadline = time.monotonic() + 3
            while time.monotonic() < deadline:
                odom.header.stamp = node.get_clock().now().to_msg()
                bev.header.stamp = odom.header.stamp
                odom_pub.publish(odom)
                grid_pub.publish(bev)
                rclpy.spin_once(node, timeout_sec=0.02)
                if any(len(t) > 15 and t[10] == 0 and t[14] == 0 for t in traces):
                    break
            assert any(len(t) > 15 and t[10] == 0 and t[14] == 0 for t in traces)
        finally:
            for process in processes:
                process.terminate()
            for process in processes:
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=5)
            node.destroy_node()
            rclpy.shutdown()
            log.seek(0)
            print(log.read())
