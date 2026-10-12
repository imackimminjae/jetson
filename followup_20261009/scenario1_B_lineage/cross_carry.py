from pathlib import Path
import ast,json,gzip,subprocess,os
import numpy as np
H=Path(__file__).resolve().parent;P=H.parent/'scenario1_sway_offline'
# Reuse the audited pure reconstruction function without rerunning the report.
tree=ast.parse((H/'analyze.py').read_text());fn=next(x for x in tree.body if isinstance(x,ast.FunctionDef) and x.name=='rebuild');exec(compile(ast.Module(body=[fn],type_ignores=[]),'rebuild','exec'))
data={}
for n in [1,5]:
    a=json.loads((P/f'input_{n}.json').read_text());j=min(range(len(a['cycles'])),key=lambda i:abs(a['cycles'][i]['y']+22.5));data[n]=dict(route=a['route'],current=a['cycles'][j],previous=a['cycles'][j-1])
env=dict(os.environ,ROS_DOMAIN_ID='177',ROS_LOCALHOST_ONLY='1',ROS_LOG_DIR=str(H/'ros_logs'),OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
sp=H/'carry_specs.json';sp.write_text(json.dumps([dict(name='line_only',fixed_ref=True,fixed_wp=True,guard=True,swap_point=True,swap_heading=True)]))
rows=[]
for grid_from in [1,5]:
    for path_from in [1,5]:
        for position_from in [1,5]:
            ref,progress,segment=rebuild(data[path_from]['previous']['rec_planned'],data[position_from]['current']);heading=np.arctan2(*(ref[6]-ref[5])[::-1])
            c=dict(data[grid_from]['current'],donor_point=ref[5].tolist(),donor_heading=float(heading));tag=f'grid{grid_from}_path{path_from}_pos{position_from}';ip=H/(tag+'.json');ip.write_text(json.dumps(dict(route=data[grid_from]['route'],cycles=[c])));op=H/(tag+'_result.json')
            with (H/(tag+'.log')).open('w') as f:subprocess.run([str(H/'line_probe'),str(ip),'/dev/stdin',str(P/'config.yaml'),str(op),str(sp)],input=gzip.decompress((P/'grids.bin.gz').read_bytes()),env=env,stdout=f,stderr=subprocess.STDOUT,check=True)
            z=json.loads(op.read_text())['line_only'][0];rows.append(dict(grid_from=grid_from,path_from=path_from,position_from=position_from,point=ref[5].tolist(),heading_deg=float(np.degrees(heading)),progress=progress,B=z['B'],candidate_step=z['B_candidate_step']))
(H/'cross_carry.json').write_text(json.dumps(rows,indent=2));print(json.dumps(rows,indent=2))
