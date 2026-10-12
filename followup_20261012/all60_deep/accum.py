import os,pickle,json,numpy as np
os.sched_setaffinity(0,{min(os.sched_getaffinity(0))})
G=pickle.load(open('grids_sparse.pkl','rb'))
X0,X1,Y0,Y1,res=-275,320,-170,260,0.25;nx,ny=int((X1-X0)/res),int((Y1-Y0)/res);occ=np.zeros((ny,nx),np.float32);cnt=np.zeros((ny,nx),np.float32)
g=.3125;GX,GY=np.meshgrid((np.arange(128)+.5)*g,-20+(np.arange(128)+.5)*g)
for t,x,y,yaw,data in G:
    dat=np.frombuffer(data,dtype=np.int8).reshape(128,128);wx=x+np.cos(yaw)*GX-np.sin(yaw)*GY;wy=y+np.sin(yaw)*GX+np.cos(yaw)*GY
    ix=((wx-X0)/res).astype(int);iy=((wy-Y0)/res).astype(int);ok=(ix>=0)&(ix<nx)&(iy>=0)&(iy<ny)&(dat>=0)
    np.add.at(occ,(iy[ok],ix[ok]),(dat[ok]>50).astype(np.float32));np.add.at(cnt,(iy[ok],ix[ok]),1)
p=np.where(cnt>0,occ/np.maximum(cnt,1),np.nan).astype(np.float32);np.save('bev_accum_20261009.npy',p);json.dump(dict(ext=[X0,X1,Y0,Y1],res=res),open('bev_accum.json','w'))
print('done',len(G))
