import time,json,os
import rclpy
from rclpy.node import Node
rclpy.init();n=Node('codex_readonly_grid_discovery',enable_rosout=False,start_parameter_services=False)
start=time.monotonic()
while time.monotonic()-start<12:rclpy.spin_once(n,timeout_sec=.2)
print(json.dumps({'domain':os.environ.get('ROS_DOMAIN_ID'),'nodes':n.get_node_names_and_namespaces(),'topics':n.get_topic_names_and_types(),'grid_publishers':[{'name':p.node_name,'namespace':p.node_namespace,'type':p.topic_type,'qos':str(p.qos_profile)} for p in n.get_publishers_info_by_topic('/bev/occupancy_grid')]},indent=2),flush=True)
n.destroy_node();rclpy.shutdown()
