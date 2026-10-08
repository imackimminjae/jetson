#!/usr/bin/env python3
"""Paper figures from recorded signals and explicitly labeled reconstructions."""
from pathlib import Path
from collections import defaultdict
import csv,gzip,json,os
from contextlib import nullcontext
os.environ.setdefault('MPLCONFIGDIR','/tmp/hil_paper_matplotlib')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib.lines import Line2D
from matplotlib.colors import ListedColormap
from matplotlib.transforms import Affine2D
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch
import numpy as np

OUT=Path(__file__).resolve().parent
REVIEW=OUT.parents[1]/'followup_20261008/batches_182344_183952_185426_review'
plt.rcParams.update({'font.size':9,'axes.titlesize':10,'axes.labelsize':9,'figure.dpi':120,'savefig.dpi':300,'pdf.fonttype':42,'ps.fonttype':42,'axes.spines.top':False,'axes.spines.right':False})
COLORS=plt.get_cmap('tab10').colors
P={};provenance=[]
RESUME=os.environ.get('HIL_FIGURE_RESUME')=='1'
for s in (1,2,3):
    p=REVIEW/f'scenario{s}';summary=json.loads((p/'summary.json').read_text());d=json.load(gzip.open(p/'telemetry.json.gz','rt'))['data']
    P[s]=dict(summary=summary,d=d,lo=np.array([v for t,v in d['/debug/lower_mpc_trace']]),ot=np.array([t for t,v in d['/motive/vehicle/odom_map']]),od=np.array([v[:6] for t,v in d['/motive/vehicle/odom_map']]),route=np.array(summary['route']),mosaic=np.load(OUT/'scenes'/f's{s}_road_mosaic.npz'))
SCENES=json.loads((OUT/'scenes/scene_manifest.json').read_text())
TRIALS=list(csv.DictReader((OUT/'tables/trials.csv').open()))

def clock(t):
    from datetime import datetime
    from zoneinfo import ZoneInfo
    return datetime.fromtimestamp(float(t),ZoneInfo('Asia/Seoul')).strftime('%H:%M:%S.%f')[:-3]
def register(name,scope,signals,method,times=''):
    provenance.append(dict(figure=f'figures/{name}',scope=scope,timestamps_ros_s=times,signals=signals,method=method))
def save(fig,name,scope,signals,method,times=''):
    for ext in ('pdf','png'):
        target=OUT/'figures'/f'{name}.{ext}'
        refresh_full=os.environ.get('HIL_FIGURE_REFRESH_FULL')=='1' and name.endswith('_full')
        if not (RESUME and target.exists() and not refresh_full):fig.savefig(target,bbox_inches='tight')
        register(f'{name}.{ext}',scope,signals,method,times)
    plt.close(fig)
    print('Figure',name,flush=True)
def road(ax,s):
    z=P[s]['mosaic'];v=z['votes'];r=z['road_votes'];pic=np.full(v.shape,2.);m=v>0;pic[m]=np.where(r[m]/v[m]>=.5,1.,0.)
    x,y=z['origin'];res=float(z['resolution']);ax.imshow(pic,origin='lower',extent=[x,x+v.shape[1]*res,y,y+v.shape[0]*res],cmap=ListedColormap(['#d8dfe2','#f8f6ec','#ffffff']),vmin=0,vmax=2,interpolation='nearest',zorder=0)
def route_axes(ax,s,attempt=None,number=False,show_fail=True):
    data=P[s];r=data['route'];road(ax,s);ax.plot(r[:,0],r[:,1],'o--',c='#2754b8',ms=3,lw=1.2,zorder=3,label='SD waypoint route')
    ats=data['summary']['attempts'];ats=[ats[attempt-1]] if attempt else ats
    bounds=[r]
    for a in ats:
        i=a['attempt'];t=data['ot'];od=data['od'];run=(t>=a['start'])&(t<a['end']);coast=(t>=a['end'])&(t<a['stop_time']);color=COLORS[(i-1)%10]
        ax.plot(od[run,0],od[run,1],c=color,lw=1.35,zorder=4,label=f"A{i:02d} {a['outcome']}");ax.plot(od[coast,0],od[coast,1],c=color,lw=.95,ls=':',zorder=3)
        ax.scatter(float(a['stop_x']),float(a['stop_y']),marker='s',s=24,c=[color],edgecolors='white',linewidths=.3,zorder=5)
        xy=od[run,:2];bounds.extend([xy,od[coast,:2]])
        if a['outcome']!='completed':ax.scatter(*xy[-1],marker='X',s=70,c='#d22228',edgecolors='white',linewidths=.4,zorder=8)
        if show_fail:
            bad=[e for _,e in data['d']['/debug/upper_branch_event'] if a['start']<=e['t_start']<a['end'] and not e['valid_plan']]
            if bad:ax.scatter([e['x'] for e in bad],[e['y'] for e in bad],marker='x',s=20,c='#d22228',lw=.7,zorder=6)
        if number:ax.annotate(str(i),(float(a['stop_x']),float(a['stop_y'])),xytext=(3,4),textcoords='offset points',fontsize=7,color=color)
    ax.scatter(*r[0],marker='^',s=85,c='#147d58',edgecolors='white',zorder=8,label='Start');ax.scatter(*r[-1],marker='*',s=130,c='#e5a300',edgecolors='white',linewidths=.4,zorder=8,label='Goal')
    for idx in range(1,len(r)-1):ax.annotate(f'W{idx}',r[idx],xytext=(4,4),textcoords='offset points',fontsize=7,color='#2754b8')
    pts=np.vstack([x for x in bounds if len(x)]);ax.set_xlim(pts[:,0].min()-12,pts[:,0].max()+12);ax.set_ylim(pts[:,1].min()-12,pts[:,1].max()+12)
    ax.set_aspect('equal');ax.set_xlabel('map X [m]');ax.set_ylabel('map Y [m]');ax.grid(alpha=.15)
    return ax

fig=plt.figure(figsize=(13,11));gs=fig.add_gridspec(2,2,height_ratios=[1.35,1])
for s,pos in [(1,gs[0,0]),(3,gs[0,1]),(2,gs[1,:])]:
    ax=fig.add_subplot(pos);route_axes(ax,s,number=True);a=P[s]['summary']['attempts'];completed=sum(z['outcome']=='completed' for z in a)
    ax.set_title(f'Scenario {s}: {completed}/10 goal approaches\n{Path(P[s]["summary"]["session"]).name}',fontsize=10)
fig.suptitle('All 30 HIL attempts — reconstructed road context',fontsize=15)
handles=[Line2D([0],[0],c='#2754b8',ls='--',marker='o',label='SD route'),Line2D([0],[0],c='k',label='Running trajectory'),Line2D([0],[0],c='k',ls=':',label='Post-decision coast'),Line2D([0],[0],marker='s',ls='',c='k',label='Recorded final stop'),Line2D([0],[0],marker='x',ls='',c='#d22228',label='Upper rejection / no candidate'),Line2D([0],[0],marker='X',ls='',c='#d22228',label='Unsuccessful termination')]
fig.legend(handles=handles,loc='lower center',ncol=3,bbox_to_anchor=(.5,.005),fontsize=9)
fig.text(.5,.064,'Background: grid-vote mosaic, aligned after recording; not an independent road boundary. W labels are route vertices.',ha='center',fontsize=8)
fig.subplots_adjust(top=.92,bottom=.12,hspace=.27)
save(fig,'01_all_routes','latest 30 attempts','odom_map; global_path; upper_branch_event; batch status; occupancy_grid','running solid; coasting dotted; grid mosaic 0.5m; >=50% drivable vote; each run own frame alignment')

# Every attempt: map plus matching telemetry; one PDF per scenario.
for s in (1,2,3):
    data=P[s];lo=data['lo'];ss=data['summary'];name=f'02_s{s}_all_10_attempts.pdf'
    with (nullcontext() if RESUME and (OUT/'figures'/name).exists() else PdfPages(OUT/'figures'/name)) as pdf:
        for a in ss['attempts'] if pdf is not None else []:
            fig=plt.figure(figsize=(12,8));gs=fig.add_gridspec(4,2,width_ratios=[1.05,1.4]);ax=fig.add_subplot(gs[:,0]);route_axes(ax,s,a['attempt'])
            row=next(r for r in TRIALS if int(r['scenario'])==s and int(r['attempt'])==a['attempt'])
            ax.set_title(f"A{a['attempt']:02d}: {a['outcome']}\n{float(row['observed_distance_running_m']):.1f} m / {float(row['running_duration_s']):.1f} s running\nFinal stop–goal: {float(row['final_stop_distance_to_goal_m']):.1f} m")
            z=lo[(lo[:,0]>=a['start'])&(lo[:,0]<a['stop_time'])];t=z[:,0]-a['start'];valid=(z[:,10]==1)|(z[:,27]==1)
            specs=[(4,'Speed [m/s]'),(6,'Lower-path lateral error [m]'),(7,'Body heading error [deg]'),(26,'Steering command [deg-equiv.]')]
            for j,(col,label) in enumerate(specs):
                aa=fig.add_subplot(gs[j,1]);y=z[:,col].copy()
                if col in (6,7):y[~valid]=np.nan
                if col in (7,26):y=np.degrees(y)
                aa.plot(t,y,lw=.95,c='#1866a5');aa.axvline(a['end']-a['start'],ls='--',c='#b82222',lw=.9);aa.axvspan(a['end']-a['start'],a['stop_time']-a['start'],color='#777',alpha=.1)
                if col==26:aa.axhline(np.degrees(.3),ls=':',c='gray');aa.axhline(-np.degrees(.3),ls=':',c='gray')
                aa.set_ylabel(label);aa.grid(alpha=.2);aa.set_xlim(0,a['stop_time']-a['start'])
                if j==3:aa.set_xlabel(f"Time from running start [s]; {clock(a['start'])} KST")
            fig.suptitle(f"Scenario {s} / Attempt {a['attempt']:02d} — {Path(ss['session']).name}",fontsize=13)
            fig.text(.5,.02,'Recorded EKF-derived map position. Gray time span / dotted route: post-decision coast. Steering is a command; no wheel-angle measurement.',ha='center',fontsize=8)
            fig.subplots_adjust(wspace=.32,hspace=.2,top=.91,bottom=.09);pdf.savefig(fig,bbox_inches='tight');plt.close(fig)
    register(name,f'scenario{s}, attempts 1–10','odom_map; lower_mpc_trace[0,4,6,7,10,12,26,27]; batch phases','all error-valid running samples; neutral missing errors shown as gaps; output command rad to deg')
    fig,axs=plt.subplots(2,5,figsize=(17,8))
    for a,ax in zip(ss['attempts'],axs.flat):route_axes(ax,s,a['attempt']);ax.set_title(f"A{a['attempt']:02d}: {a['outcome']}",fontsize=9);ax.tick_params(labelsize=7)
    fig.suptitle(f'Scenario {s} — all ten attempts (running + coast to final stop)\n{Path(ss["session"]).name}',fontsize=13);fig.tight_layout(rect=[0,.04,1,.93]);fig.text(.5,.01,'Grid mosaic is a post-hoc visualization, not road-containment ground truth. Red X: failure termination. Red ×: failed upper cycle.',ha='center',fontsize=9)
    save(fig,f'02_s{s}_all_10_routes',f'scenario{s}, all10','odom_map; grid; upper events; route','per-attempt full trajectory including coast')

legend=[Line2D([0],[0],color='#2754b8',ls='--',label='SD waypoints'),Line2D([0],[0],color='#a778c4',ls=':',label='Carried local reference'),Line2D([0],[0],color='#50a4bd',lw=2,label='Allowed interval'),Line2D([0],[0],color='#0b8050',lw=3,label='Selected interval'),Line2D([0],[0],color='#db2a35',marker='o',label='Published plan'),Line2D([0],[0],color='#db9500',ls='--',label='Recorded lower path'),Line2D([0],[0],color='#171717',label='Executed past trajectory'),Line2D([0],[0],color='#666666',ls='--',label='Later execution (hindsight)')]
groups=defaultdict(list)
for z in SCENES:groups[(z['scenario'],z['attempt'],z['group'])].append(z)
for (s,attempt,group),ss in groups.items():
    data=P[s];a=data['summary']['attempts'][attempt-1];r=data['route'];ot=data['ot'];od=data['od']
    for full in (False,True):
        count=len(ss);fig,axs=plt.subplots(2,2,figsize=(11,10));axs=axs.flat
        for ax,z in zip(axs,ss):
            e=z['event'];x,y,yaw=e['x'],e['y'],e['yaw_rad'];grid=np.load(OUT/z['grid_file'])['grid'];w,h,res,ox,oy,frame=z['grid_metadata'];g=np.where(grid<0,2,np.where(grid<=1,1,0))
            transform=Affine2D().rotate(yaw).translate(x,y)+ax.transData
            ax.imshow(g,origin='lower',extent=(ox,ox+w*res,oy,oy+h*res),transform=transform,cmap=ListedColormap(['#d1d9de','#fcfaf0','#eee5f0']),vmin=0,vmax=2,interpolation='nearest',zorder=0)
            corner=np.array([[ox,oy],[ox+w*res,oy],[ox+w*res,oy+h*res],[ox,oy+h*res],[ox,oy]]);rot=np.array([[np.cos(yaw),-np.sin(yaw)],[np.sin(yaw),np.cos(yaw)]]);corner=corner@rot.T+[x,y];ax.plot(corner[:,0],corner[:,1],c='#999',lw=.7)
            ax.plot(r[:,0],r[:,1],'--o',c='#2754b8',lw=1.1,ms=3)
            p=np.array(e['preview_world']);ax.plot(p[:,0],p[:,1],':',c='#a778c4',lw=1.5)
            sel=e['selected_corridors']
            for row in e['intervals_world']:
                k,j=int(row[0]),int(row[1]);chosen=len(sel)>k and sel[k]==j;skip=e['side_boundary_tail_start']>=2 and k>=e['side_boundary_tail_start'];c='#0b8050' if chosen and not skip else '#50a4bd'
                ax.plot([row[2],row[4]],[row[3],row[5]],c=c,lw=2.8 if chosen else 1.2,alpha=.9 if chosen else .7,ls=':' if skip else '-')
                if chosen:ax.text((row[2]+row[4])/2,(row[3]+row[5])/2,f'k{k}',fontsize=7,color='#004d30')
            lp=z['lower_path']
            if lp:
                p=np.array(lp['xy']);ax.plot(p[:,0],p[:,1],'--',c='#db9500',lw=1.9)
            plan=e['published_world'] if e['valid_plan'] else e['retained_world']
            if plan:
                p=np.array(plan);ax.plot(p[:,0],p[:,1],'-o' if e['valid_plan'] else ':o',c='#db2a35',lw=1.2,ms=3)
            mask=(ot>=a['start'])&(ot<=e['t_start']);future=(ot>e['t_start'])&(ot<a['end']);ax.plot(od[mask,0],od[mask,1],c='#171717',lw=1.8);ax.plot(od[future,0],od[future,1],'--',c='#666',lw=.9)
            # Vehicle width was not recorded; do not fabricate a footprint.
            axis=np.array([np.cos(yaw),np.sin(yaw)]);ends=np.array([x,y])+np.array([[-2.35],[2.35]])*axis
            ax.plot(ends[:,0],ends[:,1],c='k',lw=3);ax.scatter(x,y,s=28,c='k',zorder=9);ax.arrow(x,y,3*axis[0],3*axis[1],width=.15,head_width=1,color='k',length_includes_head=True)
            center=np.array([x,y])+rot@np.array([20. if full else 12.,0]);half=30 if full else 20
            ax.set_xlim(center[0]-half,center[0]+half);ax.set_ylim(center[1]-half,center[1]+half);ax.set_aspect('equal');ax.set_xlabel('map X [m]');ax.set_ylabel('map Y [m]');ax.grid(alpha=.12)
            subtitle=f"{z['panel']}. {clock(e['t_start'])} KST / plan {e['plan_seq']}\nN={e['horizon']}, max candidates={e['max_candidates']} / {e['reason']}"
            if not e['valid_plan']:subtitle+='; retained' if e['retained_previous_path'] else '; no path'
            ax.set_title(subtitle,fontsize=9)
        if count<4:axs[3].axis('off')
        fig.suptitle(f"Scenario {s} / A{attempt:02d} / {group.replace('_',' ')} — {'full grid extent' if full else 'zoom'}\n{ss[0]['run_id']}",fontsize=12)
        fig.legend(handles=legend,loc='lower center',ncol=4,bbox_to_anchor=(.5,.025),fontsize=8)
        fig.text(.5,.012,'Recorded plans/paths; grid matched retrospectively by receipt time. Black vehicle axis: length 4.7 m; width unavailable (not a footprint).',ha='center',fontsize=7.5)
        fig.subplots_adjust(top=.9,bottom=.13,hspace=.27,wspace=.2)
        name=f"03_s{s}_a{attempt:02d}_{group}_{'full' if full else 'zoom'}"
        save(fig,name,ss[0]['run_id']+f' A{attempt:02d}','occupancy_grid; upper_branch_event; global_path; lower_reference_path/with_arclength; odom_map','all times and raw grid/path association in scenes/scene_manifest.json; candidate IDs local to cycle',','.join(str(z['event']['t_start']) for z in ss))

# Full active-run histories of four explicitly selected representative outcomes.
for s,attempt in [(1,1),(1,6),(1,9),(2,1),(2,3),(3,1),(3,4)]:
    data=P[s];a=data['summary']['attempts'][attempt-1];lo=data['lo'];z=lo[(lo[:,0]>=a['start'])&(lo[:,0]<a['stop_time'])];t=z[:,0]-a['start'];valid=(z[:,10]==1)|(z[:,27]==1)
    fig,axs=plt.subplots(7,1,figsize=(11,12),sharex=True)
    cols=[(4,'Speed [m/s]'),(3,'Body heading [deg]'),(6,'Lateral error [m]'),(7,'Heading error [deg]'),(26,'Steering cmd [deg-equiv.]'),(24,'Lower solver [ms]'),(12,'Lower mode')]
    for ax,(col,label) in zip(axs,cols):
        y=z[:,col].copy()
        if col in (6,7):y[~valid]=np.nan
        if col in (3,7,26):y=np.degrees(y)
        ax.plot(t,y,lw=.9,c='#196aa5');ax.axvline(a['end']-a['start'],c='#b82222',ls='--',lw=.7);ax.set_ylabel(label);ax.grid(alpha=.2)
        if col==26:ax.axhline(np.degrees(.3),ls=':',color='gray');ax.axhline(-np.degrees(.3),ls=':',color='gray')
    es=[e for _,e in data['d']['/debug/upper_branch_event'] if a['start']<=e['t_start']<a['stop_time'] and not e['valid_plan']]
    for e in es:
        for ax in (axs[0],axs[4]):ax.axvline(e['t_start']-a['start'],c='#cf2d35',alpha=.2,lw=.5)
    scenes=[z for z in SCENES if z['scenario']==s and z['attempt']==attempt and z['group'] in ('branch','curve')]
    for scene in scenes:axs[1].axvline(scene['event']['t_start']-a['start'],c='#9863ac',ls=':',lw=.8);axs[1].text(scene['event']['t_start']-a['start'],axs[1].get_ylim()[1],f"{scene['group']}{scene['panel']}",fontsize=7,rotation=90,va='top')
    axs[-1].set_xlabel(f"Time since running start [s] ({clock(a['start'])} KST)");axs[-1].set_xlim(0,a['stop_time']-a['start']);fig.suptitle(f"Scenario {s} / A{attempt:02d}: {a['outcome']}\n{Path(data['summary']['session']).name}",fontsize=12)
    fig.text(.5,.017,'Errors are relative to the current lower path; neutral placeholder errors excluded. Red lines: upper failure. Dashed boundary: runner decision. No measured steering.',ha='center',fontsize=8)
    fig.tight_layout(rect=[0,.035,1,.95]);save(fig,f'04_s{s}_a{attempt:02d}_histories',Path(data['summary']['session']).name+f' A{attempt}','lower_mpc_trace; upper events; batch phases','recorded values; valid errors only; radians converted to degrees for plots',f"{a['start']}..{a['stop_time']}")

# Runtime distributions without mixing the measured scopes.
fig,axs=plt.subplots(2,2,figsize=(12,8))
for s in (1,2,3):
    rows=list(csv.DictReader(gzip.open(OUT/'tables'/f's{s}_cycle_times.csv.gz','rt')));times=[float(x['cycle_including_report_ms']) for x in rows if x['phase']=='running' and x['cycle_including_report_ms']]
    lo=P[s]['lo'];a=P[s]['summary']['attempts'];mask=np.zeros(len(lo),bool)
    for r in a:mask|=(lo[:,0]>=r['start'])&(lo[:,0]<r['end'])
    vals=[np.array(times),lo[mask&np.isfinite(lo[:,24]),24],lo[mask&np.isfinite(lo[:,24]),11]]
    for ax,v in zip(axs.flat,vals):
        v=np.sort(v[v>0]);ax.plot(v,np.arange(1,len(v)+1)/len(v),label=f'S{s} (n={len(v)})',c=COLORS[s-1]);ax.set_xscale('log');ax.set_ylabel('Empirical CDF');ax.set_xlabel('Measured elapsed [ms]');ax.grid(alpha=.2)
    t=np.array([float(x['start_interval_ms']) for x in rows if x['phase']=='running']);axs[1,1].plot(np.sort(t),np.arange(1,len(t)+1)/len(t),label=f'S{s}',c=COLORS[s-1])
axs[0,0].set_title('Upper cycle incl. matched report overhead (500 ms period)')
axs[0,1].set_title('Lower solver only — not entire controller (100 ms period)')
axs[1,0].set_title('Lower build to solver return — partial callback')
axs[1,1].set_title('Upper start-to-start interval — scheduling, not runtime');axs[1,1].set_xlabel('Interval [ms]');axs[1,1].set_ylabel('Empirical CDF');axs[1,1].grid(alpha=.2)
for ax in axs.flat:ax.legend()
fig.suptitle('Runtime evidence during running phases — failures included when solver was called',fontsize=13);fig.tight_layout(rect=[0,.045,1,.95]);fig.text(.5,.014,'Upper measured cycle excludes callback dispatch delay. Lower complete callback was not instrumented. Observed maxima are not WCET bounds.',ha='center',fontsize=8)
save(fig,'05_runtime_distributions','latest30, running only','upper_stage_timing; lower_mpc_trace[11,24]','CDF uses all measured calls including failed solves; report overhead matched from next cycle_seq')

# Failure/mode overview with actual relative time for every run.
fig,axs=plt.subplots(3,1,figsize=(12,10))
for s,ax in zip((1,2,3),axs):
    data=P[s];lo=data['lo']
    for a in data['summary']['attempts']:
        i=a['attempt'];dur=a['end']-a['start'];ax.plot([0,dur],[i,i],c='#148354' if a['outcome']=='completed' else '#cf4550',lw=3)
        e=[e for _,e in data['d']['/debug/upper_branch_event'] if a['start']<=e['t_start']<a['end'] and not e['valid_plan']]
        for reason,col,marker in [('solver_rejected','#e18c10','|'),('no_candidate','#b1243c','x')]:
            bad=[z for z in e if z['reason']==reason];ax.scatter([z['t_start']-a['start'] for z in bad],[i]*len(bad),marker=marker,c=col,s=28,zorder=4)
        z=lo[(lo[:,0]>=a['start'])&(lo[:,0]<a['end'])&(lo[:,12]==3)];ax.scatter(z[:,0]-a['start'],np.ones(len(z))*(i+.15),c='#4d4d4d',marker='.',s=5)
        ax.text(dur+1,i,a['outcome'],va='center',fontsize=7)
    ax.set_title(f'Scenario {s} / {Path(data["summary"]["session"]).name}');ax.set_yticks(range(1,11));ax.set_ylabel('Attempt');ax.invert_yaxis();ax.set_xlabel('Time since running start [s]');ax.grid(alpha=.15);ax.set_xlim(0,ax.get_xlim()[1]+10)
fig.suptitle('All failures and interruptions during the 30 primary trials',fontsize=13);fig.tight_layout(rect=[0,.05,1,.95]);fig.text(.5,.016,'Green/red bars: goal approach / unsuccessful termination. Orange |: rejected solve. Red ×: no candidate. Gray dots: lower neutral mode 3.',ha='center',fontsize=8)
save(fig,'06_failures_all_trials','latest30, running only','upper_branch_event; lower_trace fallback_mode; batch phase','all failed upper cycles shown, including no-solver candidate failures; neutral mode shown separately')

# Diagram depicts verified functional boundaries, never a substitute for a hardware photograph.
fig,ax=plt.subplots(figsize=(13,6));ax.axis('off');ax.set_xlim(0,13);ax.set_ylim(0,6)
def box(x,y,w,h,text,color):
    ax.add_patch(FancyBboxPatch((x,y),w,h,boxstyle='round,pad=.15',facecolor=color,edgecolor='#596471',lw=1));ax.text(x+w/2,y+h/2,text,ha='center',va='center',fontsize=10)
box(.4,3.5,3,1.5,'PHYSICAL: PX4 board\nCUAV 7 Nano (historical ID)\nSIH vehicle dynamics + virtual sensors\nEKF state estimation','#dce9f4')
box(4.9,3.7,3,1.25,'PHYSICAL: Jetson host\nMAVLink state → map alignment\nscalar yaw, filtered speed','#dce9f4')
box(9.2,3.7,3,1.25,'VIRTUAL SCENE: Isaac Sim\nRoad, vehicle, camera images\nExternal host/version unrecorded','#f5ebd6')
box(9.2,1,3,1.25,'RGB → bird’s-eye projection\nRoad mask → occupancy grid\nVehicle frame: 40 × 40 m','#f5ebd6')
box(4.9,1,3,1.25,'Jetson: local MIQP formulation\nDAQP corridor selection\nLower tracking QP → command','#dce9f4')
box(.4,1,3,1.25,'MAVLink UDP → MAVProxy\n→ USB/serial → PX4 SIH\nSteering/throttle commands','#dce9f4')
for start,end in [((3.55,4.25),(4.75,4.25)),((8.05,4.25),(9.05,4.25)),((10.7,3.55),(10.7,2.4)),((9.05,1.6),(8.05,1.6)),((4.75,1.6),(3.55,1.6)),((1.9,2.4),(1.9,3.35)),((6.4,3.55),(6.4,2.4))]:ax.add_patch(FancyArrowPatch(start,end,arrowstyle='->',mutation_scale=18,lw=1.7,color='#35424e'))
ax.text(6.4,2.85,'state',ha='center',fontsize=8);ax.text(6.4,.45,'SD-map waypoints also enter the upper planner',ha='center',fontsize=10)
fig.suptitle('HIL functional configuration — physical processors, simulated vehicle/environment',fontsize=14)
fig.text(.5,.025,'Diagram reconstructed from run configurations, startup logs and prior board identity capture. No physical road vehicle or measured wheel-angle claim.',ha='center',fontsize=9)
save(fig,'00_HIL_configuration','latest applied configs + historical board capture','effective_config; controllers.log; px4_ekf_bridge source; prior board identity','functional diagram; Isaac host and exact scene/version require additional capture')

with (OUT/'sources/figure_provenance.csv').open('w',newline='') as f:
    w=csv.DictWriter(f,fieldnames=list(provenance[0]));w.writeheader();w.writerows(provenance)
print('Figures complete:',len(provenance),flush=True)
