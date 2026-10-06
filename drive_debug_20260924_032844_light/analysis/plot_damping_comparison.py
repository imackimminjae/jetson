from pathlib import Path
import json,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
root=Path(__file__).resolve().parent;d=json.loads((root/'damping_comparison.json').read_text());t=np.array(d['t']);f,ax=plt.subplots(2,1,figsize=(11,6))
for a,begin,end,label in [(ax[0],1790188140,1790188150,'Straight section: 03:29:00–10'),(ax[1],1790188167,1790188185,'Curve section: 03:29:27–45')]:
 m=(t>=begin)&(t<end)
 for key,color,text in [('8.0','tab:blue','Existing weight 8'),('40.0','tab:orange','Revised weight 40')]:a.plot(t[m]-begin,np.degrees(np.array(d['outputs'][key])[m]),color=color,label=text)
 a.set_title(label);a.set_ylabel('Steering command (deg)');a.set_xlabel('Seconds');a.grid(alpha=.3);a.legend()
f.suptitle('Identical recorded inputs; not a new closed-loop drive');f.tight_layout();f.savefig(root/'damping_comparison.png',dpi=140)
