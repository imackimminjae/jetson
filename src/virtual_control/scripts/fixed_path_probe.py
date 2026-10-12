#!/usr/bin/env python3
"""Publish an immutable world-frame S-curve for a supervised lower-only SIH test.

Requires the lower node's batch_supervision=true. This node never arms PX4.
The caller creates OUTPUT/RUN after connecting the controller. ROS domain must
be isolated from the normal planner. All geometry is frozen before RUN exists.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path as PathMsg
from std_msgs.msg import Bool, String, Float64MultiArray
from imac_interfaces.msg import PathWithArcLength


def make_curve(peak=0.04, ds=0.1):
    """40m straight, 40m left, 20m straight, 40m right, 40m straight.

    Each turn has 10m cosine curvature ramps and a 20m constant-curvature arc.
    s is the chassis-center arc length, matching the production lower tracker.
    """
    s = np.arange(0.0, 180.0 + ds/2, ds)
    kappa = np.zeros_like(s)
    for start, sign in [(40.0, 1.0), (100.0, -1.0)]:
        u = s - start
        ramp_in = (u >= 0) & (u < 10)
        hold = (u >= 10) & (u <= 30)
        ramp_out = (u > 30) & (u <= 40)
        kappa[ramp_in] = sign*peak*(1-np.cos(np.pi*u[ramp_in]/10))/2
        kappa[hold] = sign*peak
        kappa[ramp_out] = sign*peak*(1+np.cos(np.pi*(u[ramp_out]-30)/10))/2
    heading = np.r_[0.0, np.cumsum((kappa[:-1]+kappa[1:])*ds/2)]
    mid = (heading[:-1]+heading[1:])/2
    xy = np.vstack(([0.0, 0.0], np.cumsum(ds*np.c_[np.cos(mid),np.sin(mid)],axis=0)))
    return s, xy, heading, kappa


class FixedPathProbe(Node):
    def __init__(self, output, timeout, peak):
        super().__init__('fixed_path_probe')
        self.output, self.timeout, self.peak = output, timeout, peak
        self.output.mkdir(parents=True, exist_ok=True)
        q = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_pub = self.create_publisher(PathWithArcLength, '/planner/lower_reference_path/with_arclength', q)
        self.goal_pub = self.create_publisher(Bool, '/planner/goal_reached', q)
        self.allow_pub = self.create_publisher(Bool, '/knu_batch/run_permission', 1)
        self.status_pub = self.create_publisher(String, '/fixed_path_probe/status', 10)
        self.create_subscription(Odometry, '/motive/vehicle/odom_map', self.on_pose, 50)
        self.create_subscription(String, '/px4_ekf_bridge/status', self.on_status, 10)
        self.create_subscription(Float64MultiArray, '/debug/lower_mpc_trace', self.on_trace, 100)
        self.pose = None
        self.pose_at = self.status_at = self.trace_at = -math.inf
        self.status = {}
        self.started = None
        self.created = time.monotonic()
        self.done_at = None
        self.reason = None
        self.path = None
        self.progress = 0.0
        self.records = {'pose': [], 'trace': [], 'status': []}
        self.create_timer(0.2, self.tick)

    def on_pose(self, m):
        a = [m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.orientation.z,
             m.twist.twist.linear.x, m.twist.twist.linear.y]
        if m.header.frame_id != 'map' or not all(math.isfinite(x) for x in a):
            return
        self.pose = a
        self.pose_at = time.monotonic()
        self.records['pose'].append([self.get_clock().now().nanoseconds/1e9]+a)

    def on_status(self, m):
        self.status = json.loads(m.data)
        self.status_at = time.monotonic()
        self.records['status'].append({'t':self.get_clock().now().nanoseconds/1e9, **self.status})

    def on_trace(self, m):
        self.trace_at = time.monotonic()
        self.records['trace'].append(list(m.data))

    def freeze(self):
        self.s, xy, h, self.kappa = make_curve(self.peak)
        x,y,yaw,_,_ = self.pose
        R = np.array([[math.cos(yaw),-math.sin(yaw)],[math.sin(yaw),math.cos(yaw)]])
        self.xy = xy@R.T + [x,y]
        self.path = PathWithArcLength()
        self.path.path.header.frame_id = 'map'
        self.path.s = self.s.tolist()
        for point, angle in zip(self.xy,h+yaw):
            pose = PoseStamped()
            pose.pose.position.x,pose.pose.position.y = map(float,point)
            pose.pose.orientation.z = math.sin(angle/2)
            pose.pose.orientation.w = math.cos(angle/2)
            self.path.path.poses.append(pose)
        self.digest = hashlib.sha256(self.xy.tobytes()).hexdigest()
        geometry = {'xy':self.xy.tolist(),'s':self.s.tolist(),'kappa':self.kappa.tolist(),
                    'heading':(h+yaw).tolist(),'sha256':self.digest,'finish_s':160.0}
        (self.output/'fixed_path.json').write_text(json.dumps(geometry))
        (self.output/'READY').write_text(self.digest)
        self.get_logger().info('Fixed geometry ready; waiting for RUN marker')

    def stop(self, reason):
        if self.done_at is None:
            self.reason = reason
            self.done_at = time.monotonic()
            self.get_logger().info('Probe stopped: '+reason)
            (self.output/'STOPPED').write_text(reason)
        self.allow_pub.publish(Bool(data=False))
        self.goal_pub.publish(Bool(data=True))

    def tick(self):
        now = time.monotonic()
        if self.done_at is not None:
            self.stop(self.reason)
            return
        good = (now-self.pose_at < .3 and now-self.status_at < 2.0
                and self.status.get('sih_heading_guard_state') == 'ok'
                and self.status.get('connected') is True)
        if self.path is None:
            self.allow_pub.publish(Bool(data=False))
            self.goal_pub.publish(Bool(data=True))
            if good and math.hypot(*self.pose[3:]) < .2:
                self.freeze()
            elif now-self.created > 30:
                self.stop('startup_timeout')
            return
        if self.started is None and (self.output/'RUN').exists() and good:
            self.started = now
            self.run_epoch = self.get_clock().now().nanoseconds/1e9
            (self.output/'run_start.json').write_text(json.dumps({'t':self.run_epoch}))
            self.get_logger().info('Starting fixed-path trial')
        run = self.started is not None
        if run:
            if not good:
                self.stop('pose_or_heading_guard_lost');return
            nearest = int(np.argmin(np.sum((self.xy-self.pose[:2])**2,axis=1)))
            self.progress = float(self.s[nearest])
            error = float(np.linalg.norm(self.xy[nearest]-self.pose[:2]))
            if error > 2.0:
                self.stop('cross_track_over_2m');return
            if self.progress >= 160:
                self.stop('completed');return
            if now-self.started > self.timeout:
                self.stop('run_timeout');return
            if now-self.started > 8 and self.progress < .5:
                self.stop('no_motion');return
            if now-self.started > 3 and now-self.trace_at > .5:
                self.stop('controller_trace_lost');return
        self.path.path.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(self.path)
        self.goal_pub.publish(Bool(data=not run))
        self.allow_pub.publish(Bool(data=run))
        self.status_pub.publish(String(data=json.dumps({'running':run,'progress_s':self.progress,'path_sha256':self.digest})))

    def save(self):
        self.records['result'] = {'reason':self.reason,'progress_s':self.progress,
                                  'run_start':getattr(self,'run_epoch',None)}
        (self.output/'records.json').write_text(json.dumps(self.records))


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('--timeout',type=float,default=50)
    ap.add_argument('--curvature',type=float,default=.04)
    args=ap.parse_args()
    if not 0 < args.curvature <= .07 or not 0 < args.timeout <= 90:
        ap.error('Curvature must be (0,.07], timeout (0,90]')
    rclpy.init(args=[])
    n=FixedPathProbe(args.output,args.timeout,args.curvature)
    try:
        while rclpy.ok() and (n.done_at is None or time.monotonic()-n.done_at < 1.0):
            rclpy.spin_once(n,timeout_sec=.1)
    finally:
        if rclpy.ok():n.stop(n.reason or 'interrupted')
        n.save()
        n.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__ == '__main__':
    main()
