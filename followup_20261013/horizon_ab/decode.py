# Decode the 2026-10-13 horizon A/B batches into one compact pickle per batch.
# Usage: python3 decode.py <batch_dir>...   (writes decoded_<batch>.pkl next to this file)
import collections, csv, glob, json, os, pickle, sys
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

os.sched_setaffinity(0, {min(os.sched_getaffinity(0))})
HERE = os.path.dirname(os.path.abspath(__file__))
JSON_TOPICS = {'/knu_batch/status', '/debug/upper_branch_event', '/debug/upper_stage_timing'}
TOPICS = sorted(JSON_TOPICS | {'/motive/vehicle/odom_map', '/debug/lower_mpc_trace', '/rosout'})

for batch in sys.argv[1:]:
    batch = batch.rstrip('/')
    name = os.path.basename(batch)
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=f'{batch}/bag', storage_id='mcap'),
           rosbag2_py.ConverterOptions('', ''))
    types = {x.name: get_message(x.type) for x in r.get_all_topics_and_types()}
    r.set_filter(rosbag2_py.StorageFilter(topics=[t for t in TOPICS if t in types]))
    D = collections.defaultdict(list)
    while r.has_next():
        n, b, tn = r.read_next()
        t = tn / 1e9
        m = deserialize_message(b, types[n])
        if n in JSON_TOPICS:
            try:
                D[n].append((t, json.loads(m.data)))
            except ValueError:
                pass
        elif n == '/motive/vehicle/odom_map':
            p, q, v = m.pose.pose.position, m.pose.pose.orientation, m.twist.twist.linear
            D[n].append((t, p.x, p.y, q.z, q.w, v.x, v.y))
        elif n == '/rosout':
            D[n].append((t, m.name, m.level, m.msg))
        else:
            D[n].append((t, list(m.data)))
    rows = [row for f in glob.glob(f'{batch}/results/*/summary.csv') for row in csv.DictReader(open(f))]
    cfg = open(f'{batch}/effective_config.yaml').read()
    route = sorted(glob.glob(f'{batch}/routes/*.csv'))
    out = dict(batch=name, data=dict(D), summary=rows, config=cfg,
               route=[[float(x['x']), float(x['y'])] for x in csv.DictReader(open(route[0]))] if route else None)
    pickle.dump(out, open(f'{HERE}/decoded_{name}.pkl', 'wb'))
    print(name, {k: len(v) for k, v in D.items()}, flush=True)
