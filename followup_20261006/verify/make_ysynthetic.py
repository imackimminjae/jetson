"""Clean synthetic Y-junctions modelled on scenario1 junction 2: approach heading 0, a continuing arm (+/-12 deg)
and a diverging arm (-/+40 deg). Route waypoints point to one arm; the harness judges which arm was taken."""
import numpy as np, json
RES=0.1; X0,Y0=-30.0,-100.0; W,H=2200,2000
xs=X0+(np.arange(W)+.5)*RES; ys=Y0+(np.arange(H)+.5)*RES; XX,YY=np.meshgrid(xs,ys)
def seg_dist(a,b):
    d=b-a; u=np.clip(((XX-a[0])*d[0]+(YY-a[1])*d[1])/(d@d),0,1); return np.hypot(XX-a[0]-u*d[0],YY-a[1]-u*d[1])
def build(name,cont_deg,div_deg,w_app=8.0,w_cont=7.0,w_div=7.0):
    J=np.array([80.0,0.0]); a0=np.array([-25.0,0.0])
    c=J+90*np.array([np.cos(np.radians(cont_deg)),np.sin(np.radians(cont_deg))]); d=J+90*np.array([np.cos(np.radians(div_deg)),np.sin(np.radians(div_deg))])
    free=(seg_dist(a0,J)<=w_app/2)|(seg_dist(J,c)<=w_cont/2)|(seg_dist(J,d)<=w_div/2)
    bm=np.where(free,0,100).astype(np.int8); f=f'/home/imac/ros2_ws/followup_20261006/verify/{name}.bin'; bm.tofile(f)
    return f,J,c,d
scenes=[]
def dense(pts):
    center=[];arc=[];s=0
    for a,b in zip(pts[:-1],pts[1:]):
        n=int(np.ceil(np.linalg.norm(b-a)/0.5))
        for i in range(n):
            q=a+(b-a)*i/n
            if center: s+=float(np.linalg.norm(q-np.array(center[-1])))
            center.append(q.tolist());arc.append(s)
    return center,arc
for geom,cont,div in (('Yright',12.0,-40.0),('Yleft',-12.0,40.0)):
    f,J,c,d=build(geom,cont,div)
    for target in ('diverging','continuing'):
        end=d if target=='diverging' else c
        u=(end-J)/np.linalg.norm(end-J)
        route=np.array([[0,0],[40,0],[75,0],(J+65*u).tolist()],float)
        center,arc=dense(route)
        for off,dyaw in ((0.0,0.0),(1.0,3.0),(-1.0,-3.0)):
            scenes.append({'name':f'{geom}_target_{target}_off{off:+.0f}_yaw{dyaw:+.0f}','route':route.tolist(),'center':center,'arc':arc,'widths':[200.0]*len(center),'spur':[],
              'bitmap':f,'origin':[X0,Y0],'resolution':RES,'width':W,'height':H,'start':[0.0,off,np.radians(dyaw)],'v0':0.0,'duration':30,'mode':'live',
              'meta':{'J':J.tolist(),'cont_dir':cont,'div_dir':div,'target':target}})
d=json.load(open('input_recorded.json')); d['scenes']=scenes; json.dump(d,open('input_ysynthetic.json','w'))
print(len(scenes),[s['name'] for s in scenes])
