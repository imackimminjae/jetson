"""Clean replica of scenario1 J1->J2 geometry: centerlines from the R3 track / aggregated map, realistic widths."""
import numpy as np, json, pickle
RES=0.1; X0,Y0=60.0,-90.0; W,H=1400,1700
xs=X0+(np.arange(W)+.5)*RES; ys=Y0+(np.arange(H)+.5)*RES; XX,YY=np.meshgrid(xs,ys)
def poly_mask(pts,width):
    m=np.zeros(XX.shape,bool)
    for a,b in zip(pts[:-1],pts[1:]):
        a=np.array(a,float);b=np.array(b,float);d=b-a;u=np.clip(((XX-a[0])*d[0]+(YY-a[1])*d[1])/(d@d),0,1)
        m|=np.hypot(XX-a[0]-u*d[0],YY-a[1]-u*d[1])<=width/2
    return m
SER=pickle.load(open('../runs_series.pkl','rb'));R=SER['R3_1005_2345_sc1']
sel=(R['prog']>50)&(R['prog']<126); tr=np.c_[R['mx'][sel],R['my'][sel]][::6]   # approach centerline incl. S-curve, ends near J2 mouth
J2=np.array([121.0,-7.0])
approach=np.vstack([tr[tr[:,1]<-9],J2])
north=[J2,[121.6,-1],[123,5],[126,11],[129.5,17.5],[133.5,25],[140,38],[150,58],[160,78]]
east=[J2,[130,-4],[140,-1],[150,2.5],[158,5],[180,12],[195,17]]
j1_left=[[103.5,-42],[98,-26],[95,-12],[93,0],[91,20],[89,40]]
free=poly_mask(approach,10.0)|poly_mask(north,8.0)|poly_mask(east,7.0)|poly_mask(j1_left,7.0)
bm=np.where(free,0,100).astype(np.int8); bm.tofile('replica_world.bin')
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
plt.figure(figsize=(8,10)); plt.imshow(free,origin='lower',extent=[X0,X0+W*RES,Y0,Y0+H*RES],cmap='gray')
GP=np.array([[108,-59],[110,-29],[118,-15],[173,-2]]); plt.plot(GP[:,0],GP[:,1],'r--o'); plt.plot(R['mx'],R['my'],'b',lw=.8); plt.xlim(80,200); plt.ylim(-85,80); plt.savefig('replica_world.png',dpi=70)
# scenes: start on R3 track near (109.4,-57.3)
i=int(np.argmin(np.hypot(R['mx']-109.4,R['my']+57.3))); j=i+6
x0,y0=R['mx'][i],R['my'][i]; yaw0=float(np.arctan2(R['my'][j]-y0,R['mx'][j]-x0))
def dense(pts):
    center=[];arc=[];s=0
    for a,b in zip(pts[:-1],pts[1:]):
        a=np.array(a,float);b=np.array(b,float);n=int(np.ceil(np.linalg.norm(b-a)/0.5))
        for k in range(n):
            q=a+(b-a)*k/n
            if center: s+=float(np.linalg.norm(q-np.array(center[-1])))
            center.append(q.tolist());arc.append(s)
    return center,arc
scenes=[]
for target,route in (('east(scenario1)',[[x0,y0],[110,-29],[118,-15],[173,-2]]),('north(control)',[[x0,y0],[110,-29],[118,-15],[133,25],[150,58]])):
    center,arc=dense(route)
    for off,dyaw in ((0.0,0.0),(0.8,3.0),(-0.8,-3.0)):
        n=np.array([-np.sin(yaw0),np.cos(yaw0)]); p=np.array([x0,y0])+off*n
        scenes.append({'name':f'replica_{target}_off{off:+.1f}_yaw{dyaw:+.0f}','route':route,'center':center,'arc':arc,'widths':[200.0]*len(center),'spur':[],
          'bitmap':'/home/imac/ros2_ws/followup_20261006/verify/replica_world.bin','origin':[X0,Y0],'resolution':RES,'width':W,'height':H,
          'start':[float(p[0]),float(p[1]),yaw0+np.radians(dyaw)],'v0':5.8,'duration':16,'mode':'live'})
d=json.load(open('input_recorded.json')); d['scenes']=scenes; json.dump(d,open('input_replica.json','w'))
print('start',round(x0,1),round(y0,1),round(np.degrees(yaw0),1),[s['name'] for s in scenes])
