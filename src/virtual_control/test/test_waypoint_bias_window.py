"""Window bounds, frozen plans, and nominal-progress integration."""
import json
from pathlib import Path
import sys

import pytest
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
ROOT = PACKAGE.parents[1]
sys.path.insert(0, str(PACKAGE/'scripts'))
from waypoint_bias_window import parse_bias_windows
from repeat_drive_session import upper_executable_name


def test_common_window_and_scenario_specific_windows():
    routes = PACKAGE/'data/knu_routes'
    assert parse_bias_windows(['4:4'], ['scenario1','scenario2'], routes) == {
        'scenario1':[4,4], 'scenario2':[4,4]}
    assert parse_bias_windows(['2:4:4','8:17:19'], ['scenario2','scenario8'], routes) == {
        'scenario2':[4,4], 'scenario8':[17,19]}


@pytest.mark.parametrize('specs,scenarios,prefixes', [
    (['0:2'], ['scenario2'], {}),
    (['5:5'], ['scenario2'], {}),
    (['4:3'], ['scenario2'], {}),
    (['2:4:4'], ['scenario2','scenario8'], {}),
    (['4:4','2:4:4'], ['scenario2'], {}),
    (['3:2:2'], ['scenario2'], {}),
    (['x:y'], ['scenario2'], {}),
    (['4:4'], ['scenario2'], {'scenario2':4}),
])
def test_invalid_or_missing_windows_fail_before_preparation(specs, scenarios, prefixes):
    with pytest.raises(ValueError):
        parse_bias_windows(specs, scenarios, PACKAGE/'data/knu_routes', prefixes)


def test_prepared_window_uses_separate_upper_and_nominal_route(tmp_path, monkeypatch):
    import scenario_bias_sweep as sweep
    for name in ['src','install']:
        (tmp_path/name).symlink_to(ROOT/name, target_is_directory=True)
    (tmp_path/'run_scenario_bias.sh').symlink_to(ROOT/'run_scenario_bias.sh')
    monkeypatch.setattr(sweep, 'W', tmp_path)
    assert sweep.main(['2','--bias','-5','0','--runs','3','--bias-waypoints','4:4','--prepare-only']) == 0
    plan=json.loads(next((tmp_path/'parameter_sweeps').glob('*/plan.json')).read_text())
    assert plan['bias_mode']=='waypoint_window'
    assert plan['planned_attempts']==6
    assert plan['bias_waypoints']=={'scenario2':[4,4]}
    for job in plan['jobs']:
        c=yaml.safe_load(Path(job['config_file']).read_text())
        u=c['upper_planner_node']['ros__parameters']
        assert u['waypoint_bias_mode']=='waypoint_window'
        assert u['waypoint_bias_start_wp']==u['waypoint_bias_end_wp']==4
        assert upper_executable_name(c)=='upper_planner_waypoint_bias_node'
        assert c['knu_batch_runner']['ros__parameters']['road_enable_waypoint_bias'] is False
    assert upper_executable_name({'upper_planner_node':{'ros__parameters':{}}})=='upper_planner_node'


@pytest.mark.parametrize('executable',['upper_planner_node','upper_planner_waypoint_bias_node'])
def test_launch_selects_requested_upper_only(executable,tmp_path,monkeypatch):
    monkeypatch.setenv('ROS_LOG_DIR',str(tmp_path/'launch_logs'))
    import importlib.util
    from launch import LaunchContext
    from launch.utilities import perform_substitutions, normalize_to_list_of_substitutions
    from launch_ros.actions import Node
    spec=importlib.util.spec_from_file_location('window_launch',PACKAGE/'launch/scenario3_repeat.launch.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    nodes=[n for n in module.generate_launch_description().entities if isinstance(n,Node)]
    context=LaunchContext();context.launch_configurations['upper_executable']=executable
    assert perform_substitutions(context,normalize_to_list_of_substitutions(nodes[2].node_executable))==executable
    assert nodes[3].node_executable=='lower_tracking_mpc_node'


@pytest.mark.skipif(__import__('os').environ.get('WAYPOINT_BIAS_ROS_TEST') != '1',
                    reason='opt-in isolated upper-only ROS integration')
@pytest.mark.parametrize('angle',[0,10,-10])
def test_actual_upper_enters_and_leaves_window_without_reset(tmp_path,angle):
    import math, os, signal, subprocess, time
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from nav_msgs.msg import Odometry, OccupancyGrid, Path as PathMsg
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import String
    assert os.environ.get('ROS_DOMAIN_ID') not in (None,'0','1')
    upper=yaml.safe_load((PACKAGE/'config/scenario_bias_baseline.yaml').read_text())['upper_planner_node']['ros__parameters']
    upper.update(enable_waypoint_bias=True,waypoint_bias_mode='waypoint_window',
                 waypoint_bias_start_wp=3,waypoint_bias_end_wp=3,
                 waypoint_yaw_bias_rad=math.radians(angle),upper_use_vehicle_rollout=False,
                 upper_r_steering_delta=0.0,
                 odom_yaw_is_orientation_z=False,grid_positive_is_drivable=True,
                 grid_value_threshold=1,require_grid_map=True)
    config=tmp_path/'upper.yaml';config.write_text(yaml.safe_dump({'upper_planner_node':{'ros__parameters':upper}}))
    rclpy.init();node=Node('waypoint_window_test');events=[]
    odom_pub=node.create_publisher(Odometry,'/motive/vehicle/odom_map',10)
    grid_pub=node.create_publisher(OccupancyGrid,'/bev/occupancy_grid',10)
    path_pub=node.create_publisher(PathMsg,'/navigation/global_path',QoSProfile(
        depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE))
    node.create_subscription(String,'/debug/upper_branch_event',lambda m:events.append(json.loads(m.data)),100)
    log=(tmp_path/'upper.log').open('w')
    process=subprocess.Popen([str(ROOT/'build/virtual_control/upper_planner_waypoint_bias_node'),
                              '--ros-args','--params-file',str(config)],stdout=log,stderr=log)
    path=PathMsg();path.header.frame_id='map'
    for x in [0.,20.,40.,60.,80.]:
        point=PoseStamped();point.pose.position.x=x;point.pose.orientation.w=1.;path.poses.append(point)
    grid=OccupancyGrid();grid.header.frame_id='Vehicle';grid.info.resolution=.5
    grid.info.width=200;grid.info.height=120;grid.info.origin.position.x=-30.;grid.info.origin.position.y=-30.
    grid.info.origin.orientation.w=1.;grid.data=[100]*(200*120)
    observed=[]
    try:
        for x,index,active in [(0.,1,False),(15.,2,True),(35.,3,False)]:
            deadline=time.monotonic()+12;matched=None
            while time.monotonic()<deadline:
                assert process.poll() is None,(tmp_path/'upper.log').read_text()
                stamp=node.get_clock().now().to_msg()
                pose=Odometry();pose.header.frame_id='map';pose.header.stamp=stamp
                pose.pose.pose.position.x=x;pose.pose.pose.orientation.w=1.
                odom_pub.publish(pose);grid.header.stamp=stamp;grid_pub.publish(grid)
                path.header.stamp=stamp;path_pub.publish(path)
                rclpy.spin_once(node,timeout_sec=.05)
                matched=next((e for e in reversed(events) if e['wp0']==index and abs(e['x']-x)<.01),None)
                if matched:break
            assert matched,(x,(tmp_path/'upper.log').read_text())
            assert matched['waypoint_bias_active'] is active
            wp1_nominal=path.poses[index+1].pose.position.x
            if active:
                distance=wp1_nominal-x
                assert matched['planning_wp1_world']==pytest.approx([
                    x+distance*math.cos(math.radians(angle)),distance*math.sin(math.radians(angle))])
            else:
                assert matched['planning_wp1_world']==pytest.approx([wp1_nominal,0.])
            observed.append(matched)
        assert len({e['path_revision'] for e in observed})==1
    finally:
        process.send_signal(signal.SIGINT)
        try:process.wait(timeout=10)
        except subprocess.TimeoutExpired:process.kill();process.wait()
        log.close();node.destroy_node();rclpy.shutdown()
