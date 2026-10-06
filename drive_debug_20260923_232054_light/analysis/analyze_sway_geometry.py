# Reuse the independently checked path reconstruction without rerunning QPs.
from pathlib import Path
exec(Path(__file__).with_name('analyze_sway_replay.py').read_text().split('N=30;')[0])
from datetime import datetime
stats=[]
for i,x in enumerate(l):
 p=paths[indices[i]];j,ey,eps,k=ref(x,p);stats.append([x[0],max(abs(k)),np.sum(abs(k)>np.tan(.3)/2.8),np.max(abs(np.diff(p['psi']))),j])
stats=np.array(stats);roi=(l[:,0]>1790173287)&(l[:,0]<1790173306)
print('future curvature abs median/p95/max whole',np.quantile(stats[:,1],[.5,.95,1]),'corner',np.quantile(stats[roi,1],[.5,.95,1]));print('cycles with future curvature > physical limit whole/corner',np.mean(stats[:,2]>0),np.mean(stats[roi,2]>0))
for i in np.argsort(stats[:,1])[-5:]:
 x=l[i];p=paths[indices[i]];j,ey,eps,k=ref(x,p);print('PEAK',datetime.fromtimestamp(x[0]).strftime('%H:%M:%S.%f')[:12], 'curvature',max(abs(k)),'steer needed deg',np.degrees(np.arctan(2.8*max(abs(k)))),'futureseconds',np.argmax(abs(k))*.1,'path',int(indices[i]))
# Heading at the same current pose on new and previous paths; no vehicle motion.
trans=[]
for i in range(1,len(l)):
 if indices[i]==indices[i-1]:continue
 x=l[i];a=ref(x,paths[indices[i-1]]);b=ref(x,paths[indices[i]]);trans.append([x[0],np.degrees(wrap(a[2]-b[2])),b[1]-a[1],a[1],b[1]])
trans=np.array(trans);print('REFRESH same-pose heading change deg p95/max',np.quantile(abs(trans[:,1]),[.95,1]),'lateral change p95/max',np.quantile(abs(trans[:,2]),[.95,1]))
for row in sorted(trans,key=lambda x:abs(x[1]),reverse=True)[:6]:print('HEADING JUMP',datetime.fromtimestamp(row[0]).strftime('%H:%M:%S.%f')[:12],row[1:])
np.savez(root/'sway_geometry.npz',curvature=stats,transitions=trans)
# Save complete reconstruction objects for plotting.
pickle.dump(paths,(root/'reconstructed_paths.pkl').open('wb'))
