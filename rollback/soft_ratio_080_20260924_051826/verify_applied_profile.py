import os
import resource
import signal
import subprocess
from pathlib import Path
import rclpy
from rcl_interfaces.srv import GetParameters

resource.setrlimit(resource.RLIMIT_CORE, (0, 0))
root = Path('/home/imac/ros2_ws')
exe = root / 'install/virtual_control/lib/virtual_control/upper_planner_node'
profile = root / 'install/virtual_control/share/virtual_control/config/tracking_control_split.yaml'
cmd = [str(exe), '--ros-args', '--params-file', str(profile)]
assert os.environ['ROS_DOMAIN_ID'] == '178'
assert os.environ['ROS_LOCALHOST_ONLY'] == '1'
rclpy.init()
node = rclpy.create_node('soft_ratio_profile_check')
with open('/tmp/soft_ratio080_startup.log', 'w') as log:
    process = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT)
    try:
        client = node.create_client(GetParameters, '/upper_planner_node/get_parameters')
        assert client.wait_for_service(timeout_sec=8.0), 'parameter service unavailable'
        request = GetParameters.Request()
        request.names = ['preview_interval_soft_ratio', 'preview_interval_boundary_margin_m']
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
        assert future.done(), 'parameter read timed out'
        values = [v.double_value for v in future.result().values]
        assert values == [.8, 2.0], values
        assert process.poll() is None
        print('PASS: installed node loads soft_ratio=0.8 and OFF margin=2.0')
    finally:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
        node.destroy_node()
        rclpy.shutdown()

for ratio in ['0.5', '1.01']:
    result = subprocess.run(cmd + ['-p', 'preview_interval_soft_ratio:=' + ratio],
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=8)
    assert result.returncode != 0
    assert 'preview_interval_soft_ratio must be finite and in (0.5, 1.0]' in result.stdout
    print('PASS: rejects invalid ratio=' + ratio)
