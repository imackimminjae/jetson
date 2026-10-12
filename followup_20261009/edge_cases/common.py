import os;os.sched_setaffinity(0,{0});os.nice(19)
import numpy as np,json,pickle,gzip,matplotlib;matplotlib.use('Agg');import matplotlib.pyplot as plt
from matplotlib import font_manager
font_manager.fontManager.addfont('/usr/share/fonts/truetype/nanum/NanumGothic.ttf');plt.rcParams.update({'font.family':'NanumGothic','axes.unicode_minus':False,'font.size':11})
S1='/tmp/claude-1000/-home-imac-ros2-ws/0001aebe-4346-4d43-92c3-665e5620a1a1/scratchpad/s1osc/data.pkl'
BC='/home/imac/ros2_ws/followup_20261009/s1_block_compare/'
def smap(prefix=''):
    m=json.load(open(BC+prefix+'static_map.json'));B=np.fromfile(BC+prefix+'static_map.bin',dtype=np.int8).reshape(m['height'],m['width'])
    return B,(m['origin'][0],m['origin'][0]+m['width']*m['res'],m['origin'][1],m['origin'][1]+m['height']*m['res'])
def bg(ax,prefix=''):
    B,e=smap(prefix);ax.imshow(np.where(B==0,1.0,0.35),origin='lower',extent=e,cmap='gray',vmin=0,vmax=1)
def note(fig,txt): fig.text(0.01,0.005,txt,fontsize=10,color='#333',va='bottom')
