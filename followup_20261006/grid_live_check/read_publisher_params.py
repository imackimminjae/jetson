import rclpy,json,time
from pathlib import Path
from rclpy.node import Node
from rcl_interfaces.srv import ListParameters,GetParameters
from datetime import datetime
from zoneinfo import ZoneInfo
h=Path(__file__).resolve().parent
rclpy.init();n=Node('codex_readonly_bev_parameters',enable_rosout=False,start_parameter_services=False)
c=n.create_client(ListParameters,'/rgb_to_occupancy/list_parameters');out={}
if not c.wait_for_service(timeout_sec=5):out={'status':'parameter_service_not_visible'}
else:
 req=ListParameters.Request();req.depth=0;f=c.call_async(req);rclpy.spin_until_future_complete(n,f,timeout_sec=5)
 if not f.done() or not f.result():out={'status':'parameter_list_timeout'}
 else:
  names=f.result().result.names;names=[v for v in names if not any(s in v.lower() for s in ['password','secret','token','credential'])];out={'names':names};g=n.create_client(GetParameters,'/rgb_to_occupancy/get_parameters')
  if g.wait_for_service(timeout_sec=3):
   q=GetParameters.Request();q.names=names;f=g.call_async(q);rclpy.spin_until_future_complete(n,f,timeout_sec=5)
   if f.done() and f.result():
    attrs={1:'bool_value',2:'integer_value',3:'double_value',4:'string_value',5:'byte_array_value',6:'bool_array_value',7:'integer_array_value',8:'double_array_value',9:'string_array_value'};values={}
    for key,val in zip(names,f.result().values):
     value=getattr(val,attrs[val.type]) if val.type in attrs else None
     if val.type>=5:value=list(value)
     values[key]=value
    out['status']='read';out['values']=values
n.destroy_node();rclpy.shutdown();p=h/('publisher_params_'+datetime.now(ZoneInfo('Asia/Seoul')).strftime('%Y%m%d_%H%M%S')+'.json');p.write_text(json.dumps(out,indent=2));print(str(p));print(json.dumps(out,indent=2))
