from pathlib import Path
import sqlite3,pickle,json,numpy as np
from rclpy.serialization import deserialize_message
from imac_interfaces.msg import PathWithArcLength
here=Path(__file__).resolve().parent;root=here.parents[1]
series=pickle.load((here/'series.pkl').open('rb'));out={}
for tag in ['050536','050627']:
    folder=root/f'drive_debug_20261006_{tag}_light'; paths=[]
    with sqlite3.connect('file:'+str(folder/'bag/bag_0.db3')+'?mode=ro',uri=True) as c:
        for t,b in c.execute("select messages.timestamp,messages.data from messages join topics on topics.id=messages.topic_id where topics.name='/planner/lower_reference_path/with_arclength' order by messages.timestamp"):
            m=deserialize_message(b,PathWithArcLength);paths.append({'t':t/1e9,'xy':[[p.pose.position.x,p.pose.position.y] for p in m.path.poses],'s':list(m.s)})
    ts=np.array([p['t'] for p in paths]);samples=[]
    for tr,p in zip(series[tag]['lower'],series[tag]['lower_progress']):
        if tr[10]!=1 or not 88<=p<=190:continue
        i=int(np.searchsorted(ts,tr[0],side='right'))-1
        if i<1:continue
        samples.append({'trace':tr.tolist(),'path_index':i,'age':float(tr[0]-ts[i])})
    out[tag]={'config':str(folder/'config_snapshot.yaml'),'paths':paths,'samples':samples}
(here/'probe_input.json').write_text(json.dumps(out)+'\n')
print({k:len(v['samples']) for k,v in out.items()})
