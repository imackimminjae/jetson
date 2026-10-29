"""Design four continuous additional routes from the supplied road network."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path

os.environ.setdefault('MPLCONFIGDIR', '/tmp/long_scenarios_mpl')
import numpy as np
from scipy.ndimage import gaussian_filter1d, map_coordinates, distance_transform_edt

ROOT=Path(__file__).resolve().parents[3]
PACKAGE=Path(__file__).resolve().parents[1]
DESIGNS={
    22: dict(label='서쪽 → 중앙 → 대각선 → 동쪽',color='#222222',
        start_observed_center_shift_m=2.4,
        edges=[21,23,28,31,34,39,-38,-35,20,-18,-15,-13,14,19],
        cuts=[[-215.3,75.1],[-121.8,41.3],[122.4,1.6],[191.4,130.4]],
        sections=['서측 교차로 접근','서측 회전교차로 통과','중앙 연결도로·남동 분기','대각선 연결도로','북동 회전교차로·동측 출구']),
    23: dict(label='북서쪽 지선 → 중앙 → 동남쪽 지선',color='#12a04b',
        edges=[10,26,27,31,34,39,-38,-37],
        cuts=[[-121.8,41.3],[77.5,9.4],[120.3,-10.3]],
        sections=['북서 지선·서측 회전교차로','중앙 연결도로','남동 분기 통과','동남 지선 출구']),
    24: dict(label='남쪽 → 회전교차로 서쪽 → 북쪽',color='#a51d3b',
        reverse_direction=True,
        edges=[0,2,4,9,16,-20,35,38,-41,42],
        observed_mask=True,center_margin_m=2.3,terminal_reserve_m=50.,
        cuts=[[105.5,-44.1],[122.4,1.6],[191.4,130.4]],
        sections=['남측 진입','남동 분기·대각선 진입','대각선 연결도로','북동 회전교차로·북측 출구']),
    25: dict(label='남서쪽 → 중앙 → 대각선 → 북쪽',color='#2276c5',
        edges=[-30,31,34,39,-38,-35,20,-18,-17,-9,-4,-2,'r0'],
        cuts=[[-121.8,41.3],[77.5,9.4],[122.4,1.6],[191.4,130.4]],
        sections=['남서 지선·서측 회전교차로','중앙 연결도로','남동 분기·대각선 진입','대각선 연결도로','북동 회전교차로·북측 출구']),
}


def arc(points):
    return np.r_[0.,np.cumsum(np.linalg.norm(np.diff(points,axis=0),axis=1))]


def simplify(points,tolerance=2.5,max_spacing=100.):
    station=arc(points);indices=[0]
    while indices[-1]<len(points)-1:
        i=indices[-1];j=i+2
        while j<len(points):
            v=points[j]-points[i];length=np.linalg.norm(v)
            offsets=points[i:j+1]-points[i]
            error=np.abs(offsets[:,0]*v[1]-offsets[:,1]*v[0])/max(length,1e-9)
            if station[j]-station[i]>max_spacing or error.max()>tolerance:break
            j+=1
        indices.append(min(len(points)-1,max(i+1,j-1)))
    return np.round(points[indices],3)


def refine_body_clearance(dense, route_margin):
    """Refine the analytical reference for the forward-facing vehicle body."""
    for _ in range(200):
        direction=np.gradient(dense,axis=0)
        direction/=np.linalg.norm(direction,axis=1)[:,None]
        normal=np.c_[-direction[:,1],direction[:,0]]
        samples=np.array([dense+direction*l+normal*w
            for l in [-2.35,0.,3.8] for w in [-.9,0.,.9]])
        margins=route_margin(samples.reshape(-1,2)).reshape(9,-1)
        worst=samples[margins.argmin(axis=0),np.arange(len(dense))]
        gradient=np.c_[route_margin(worst+[.2,0])-route_margin(worst-[.2,0]),
                       route_margin(worst+[0,.2])-route_margin(worst-[0,.2])]/.4
        correction=.5*np.maximum(0.,.3-margins.min(axis=0))[:,None]*gradient/np.maximum(np.linalg.norm(gradient,axis=1)[:,None],.1)
        dense+=gaussian_filter1d(correction,4/.25,axis=0,mode='nearest')
    return dense


def refine_reversed_reference(dense, route_margin):
    """Check the asymmetric front/rear body after reversing an analytical line.

    Mission waypoints stay an exact reversal. Small, smooth local shifts apply
    only to the illustrative dense road reference, not the driven trajectory.
    """
    station=arc(dense)
    def margins(points):
        direction=np.gradient(points,axis=0)
        direction/=np.linalg.norm(direction,axis=1)[:,None]
        normal=np.c_[-direction[:,1],direction[:,0]]
        samples=np.array([points+direction*l+normal*w
            for l in [-2.35,0.,3.8] for w in [-.9,0.,.9]])
        return route_margin(samples.reshape(-1,2)).reshape(9,-1)
    for _ in range(40):
        current=margins(dense)
        if current.min()>.025:break
        loss=float(np.square(np.maximum(.15-current,0.)).sum())
        index=np.unravel_index(current.argmin(),current.shape)[1]
        window=np.exp(-.5*((station-station[index])/8.)**2)
        window*=np.minimum(station/15.,1.)*np.minimum((station[-1]-station)/15.,1.)
        best_loss,best=loss,dense
        for step in [.05,.1,.2]:
            for angle in np.arange(0.,2*np.pi,np.pi/4):
                candidate=dense+window[:,None]*step*np.array([np.cos(angle),np.sin(angle)])
                candidate_loss=float(np.square(np.maximum(.15-margins(candidate),0.)).sum())
                if candidate_loss<best_loss:
                    best_loss,best=candidate_loss,candidate
        if best_loss>=loss-1e-8:break
        dense=best
    return dense


def generate(reference):
    reference=Path(reference).resolve()
    d=np.load(reference);xy=d['map_xy'];edge=d['edge'];aff=d['affine'];inv=np.linalg.inv(aff[:2])
    scale,bx,by=d['image_params'];signed=d['signed_distance_px']
    def pixels(points):
        q=(points-aff[2])@inv
        return np.c_[scale*q[:,0]+bx,-scale*q[:,1]+by]
    def margin(points):
        p=pixels(points)
        return map_coordinates(signed,[p[:,1],p[:,0]],order=1,mode='constant',cval=-1000)/scale
    # The west passage is confirmed by recorded Isaac drives. The supplied
    # image's narrow triangle differs there, so use the observed BEV geometry
    # for this route rather than moving it to the rejected east detour.
    observed_path=ROOT/'diagnostics/isaac_observed_roads_20261025/shared_isaac_road_mosaic.npz'
    observed=np.load(observed_path);road=observed['road'];resolution=float(observed['resolution_m'])
    observed_signed=(distance_transform_edt(road)-distance_transform_edt(~road))*resolution
    observed_low=observed['map_low']
    def observed_margin(points):
        p=(points-observed_low)/resolution-.5
        return map_coordinates(observed_signed,[p[:,1],p[:,0]],order=1,mode='constant',cval=-1000)
    result={}
    for number,design in DESIGNS.items():
        route_margin=observed_margin if design.get('observed_mask') else margin
        parts=[]
        for identifier in design['edges']:
            reverse=identifier=='r0' or (isinstance(identifier,int) and identifier<0)
            ident=0 if identifier=='r0' else abs(identifier)
            q=xy[edge==ident];q=q[::-1] if reverse else q
            if parts:
                assert np.linalg.norm(parts[-1][-1]-q[0])<.5
            parts.append(q if not parts else q[1:])
        raw=np.vstack(parts);station=arc(raw)
        terminal_reserve=design.get('terminal_reserve_m',35.)
        samples=np.arange(10.,station[-1]-terminal_reserve,.25)
        dense=np.c_[np.interp(samples,station,raw[:,0]),np.interp(samples,station,raw[:,1])]
        dense=gaussian_filter1d(dense,10/.25,axis=0,mode='nearest')
        # Spread small boundary corrections longitudinally to avoid raster-shaped
        # zigzags. These checks refer to the supplied mask, not Isaac wall truth.
        for _ in range(100):
            m=route_margin(dense)
            gradient=np.c_[route_margin(dense+[.3,0])-route_margin(dense-[.3,0]),
                           route_margin(dense+[0,.3])-route_margin(dense-[0,.3])]/.6
            correction=.7*np.maximum(0.,design.get('center_margin_m',1.5)-m)[:,None]*gradient/np.maximum(np.linalg.norm(gradient,axis=1)[:,None],.1)
            dense+=gaussian_filter1d(correction,7/.25,axis=0,mode='nearest')
        if design.get('observed_mask'):
            # Refine the narrow west connector using the whole reference body,
            # including the front overhang, rather than its center alone.
            dense=refine_body_clearance(dense,route_margin)
        points=simplify(dense)
        start_adjustment=None
        original_velocity=dense[1]-dense[0]
        original_heading=float(math.atan2(original_velocity[1],original_velocity[0]))
        if design.get('start_observed_center_shift_m'):
            old_start=points[0].copy()
            normal=np.array([-math.sin(original_heading),math.cos(original_heading)])
            points[0]=np.round(old_start+design['start_observed_center_shift_m']*normal,3)
            shift=points[0]-old_start
            # Only the first mission waypoint changes; taper the illustrative
            # dense reference back to its original line over the first 30 m.
            taper=.5*(1+np.cos(np.pi*np.minimum(arc(dense)/30.,1.)))
            dense+=taper[:,None]*shift
            start_adjustment=dict(previous_start=old_start.tolist(),shift_xy_m=shift.tolist(),
                distance_m=float(np.linalg.norm(shift)),basis='observed Isaac road center',date='2026-10-30')
        if design.get('reverse_direction'):
            points=points[::-1].copy()
            dense=refine_reversed_reference(dense[::-1].copy(),route_margin)
        velocity=np.gradient(dense,axis=0);acceleration=np.gradient(velocity,axis=0)
        curvature=np.abs(velocity[:,0]*acceleration[:,1]-velocity[:,1]*acceleration[:,0])/np.maximum(np.linalg.norm(velocity,axis=1)**3,1e-9)
        radius=float((1/np.maximum(curvature[20:-20],1e-9)).min())
        direction=velocity/np.linalg.norm(velocity,axis=1)[:,None];normal=np.c_[-direction[:,1],direction[:,0]]
        footprint=np.vstack([dense+direction*l+normal*w for l in [-2.35,0.,3.8] for w in [-.9,0.,.9]])
        footprint_margin=float(route_margin(footprint).min())
        assert radius>6.5,(number,radius)
        assert footprint_margin>(0. if design.get('observed_mask') else .15),(number,footprint_margin)
        assert np.linalg.norm(np.diff(points,axis=0),axis=1).max()<100.01
        length=float(arc(points)[-1]);heading=float(math.atan2(direction[0,1],direction[0,0]))
        if start_adjustment:heading=original_heading
        boundaries=[0]
        for target in design['cuts']:
            idx=int(np.argmin(np.linalg.norm(points-np.array(target),axis=1)))
            assert idx>boundaries[-1],(number,idx)
            boundaries.append(idx)
        boundaries.append(len(points)-1)
        sections=[dict(name=name,start_wp=boundaries[i]+1,end_wp=boundaries[i+1]+1,
                       length_m=float(arc(points)[boundaries[i+1]]-arc(points)[boundaries[i]]))
                  for i,name in enumerate(design['sections'])]
        spawn_direction=np.array([[math.cos(heading),math.sin(heading)]])
        spawn_normal=np.c_[-spawn_direction[:,1],spawn_direction[:,0]]
        spawn_samples=np.vstack([points[:1]+spawn_direction*l+spawn_normal*w for l in [-2.35,0.,3.8] for w in [-.9,0.,.9]])
        if not start_adjustment and not design.get('reverse_direction'):
            spawn_samples=np.vstack([dense[:1]+direction[:1]*l+normal[:1]*w for l in [-2.35,0.,3.8] for w in [-.9,0.,.9]])
        spawn_margin=observed_margin if start_adjustment or design.get('observed_mask') else route_margin
        source_edges=design['edges']
        if design.get('reverse_direction'):
            source_edges=[0 if e=='r0' else ('r0' if e==0 else -e) for e in reversed(source_edges)]
        metadata=dict(scenario=f'scenario{number}',label=design['label'],color=design['color'],
            length_m=round(length,3),reference_length_m=round(float(arc(dense)[-1]),3),
            waypoints=len(points),nominal_driving_sec_at_6mps=round(float(arc(dense)[-1])/6,1),
            waypoint_sampling=dict(max_spacing_m=100.,automatic_reference_deviation_tolerance_m=2.5,
                purpose='Sparse SDMap guidance; planner follows local drivable geometry between waypoints.'),
            dense_min_radius_m=round(radius,3),reference_mask_center_margin_m=float(route_margin(dense).min()),
            reference_mask_source=str(observed_path) if design.get('observed_mask') else str(reference),
            reference_mask_footprint_margin_m=footprint_margin,
            spawn_mask_footprint_margin_m=float(spawn_margin(spawn_samples).min()),
            start=points[0].tolist(),end=points[-1].tolist(),heading_rad=heading,
            source_edges=source_edges,terminal_network_reserve_m=terminal_reserve,
            continuous=True,sections=sections,actual_drive_validated=False)
        if start_adjustment:metadata['start_adjustment']=start_adjustment
        if start_adjustment or design.get('observed_mask'):metadata['spawn_mask_source']=str(observed_path)
        if design.get('reverse_direction'):
            metadata['direction_reversed_on']='2026-10-30'
            metadata['initial_network_reserve_m']=terminal_reserve
            metadata['terminal_network_reserve_m']=10.
        result[number]=dict(points=points,dense=dense,metadata=metadata)
    return result,pixels


def write_routes(result):
    directory=PACKAGE/'data/knu_routes'
    starts_path=directory/'knu_start_poses.json';starts=json.loads(starts_path.read_text())
    starts['starts']=[r for r in starts['starts'] if r['scenario'] not in [f'scenario{n}' for n in DESIGNS]]
    header=['#pragma once','#include "virtual_control/px4_odom_map_transform.hpp"',
            'namespace virtual_control { namespace px4_odom_map {',
            'inline std::optional<RouteAnchor> longRouteAnchorForName(const std::string & route_name)', '{']
    for number,item in result.items():
        points=item['points'];m=item['metadata'];name=m['scenario']
        np.savetxt(directory/f'{name}.csv',np.c_[points,np.zeros(len(points))],delimiter=',',header='x,y,z',comments='',fmt='%.3f')
        position=[*points[0],0.];heading=m['heading_rad'];second=[position[0]+5*math.cos(heading),position[1]+5*math.sin(heading),0.]
        adjustment=m.get('start_adjustment')
        starts['starts'].append(dict(scenario=name,position=position,path_first=position,
            heading_rad=heading,ros_heading_rad=heading,
            original_position=[*adjustment['previous_start'],0.] if adjustment else position,
            corrected=bool(adjustment),
            reference_mask_body_on_road=True,reference_mask_clearance_m=m['spawn_mask_footprint_margin_m'],
            validation=('observed Isaac BEV geometry; actual drive pending' if m.get('spawn_mask_source')
                        else 'supplied road mask geometry; actual Isaac drive pending'),profile='long4'))
        a=', '.join(f'{v:.12f}' for v in position);b=', '.join(f'{v:.12f}' for v in second)
        header.extend([f'  if (route_name == "{name}") {{',f'    return RouteAnchor{{{{{a}}}, {{{b}}}}};','  }'])
    header.extend(['  return std::nullopt;','}','} }  // namespace virtual_control::px4_odom_map',''])
    (PACKAGE/'include/virtual_control/knu_long_route_anchors.hpp').write_text('\n'.join(header))
    starts['version']=5
    starts['additional_profile']='long4: scenarios22-25; original starts1-21 retained'
    starts_path.write_text(json.dumps(starts,ensure_ascii=False,indent=2)+'\n')
    p=directory/'knu_batch_profile.json';profile=json.loads(p.read_text())
    profile['additional_routes']=list(dict.fromkeys(profile.get('additional_routes',[])+[f'scenario{n}' for n in DESIGNS]))
    profile['additional_profiles']={'long4':'knu_long_profile.json'}
    p.write_text(json.dumps(profile,ensure_ascii=False,indent=2)+'\n')
    long=dict(version=1,profile='long4',coordinate_frame='map',routes=[f'scenario{n}' for n in DESIGNS],
        route_count=4,default_batch_unchanged=True,sections_are_analysis_only=True,
        policy='One continuous drive per route; no stops/resets at section boundaries. '
               'Waypoints are sparse SDMap guidance, not a vehicle trajectory. '
               'Mask clearance and curvature checks apply to the dense reference, not waypoint chords. '
               'Actual Isaac drives not yet validated.',
        definitions=[item['metadata'] for item in result.values()])
    (directory/'knu_long_profile.json').write_text(json.dumps(long,ensure_ascii=False,indent=2)+'\n')


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--write',action='store_true')
    args=parser.parse_args()
    reference=ROOT/'diagnostics/map_coverage_20261025_road_reference/road_reference.npz'
    result,pixels=generate(reference)
    output=ROOT/'diagnostics/long_routes_20261027';output.mkdir(exist_ok=True)
    if args.write:write_routes(result)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.font_manager import FontProperties
    from PIL import Image
    plt.rcParams['font.family']=FontProperties(fname='/usr/share/fonts/truetype/nanum/NanumGothic.ttf').get_name()
    plt.rcParams['axes.unicode_minus']=False
    registration=json.loads(reference.with_name('registration.json').read_text())
    image=np.array(Image.open(registration['source_png']))
    fig,axes=plt.subplots(2,2,figsize=(17,12),constrained_layout=True)
    overview,axall=plt.subplots(figsize=(16,11),constrained_layout=True)
    axall.imshow(image);axall.axis('off')
    for (number,item),ax in zip(result.items(),axes.flat):
        m=item['metadata'];p=pixels(item['dense']);q=pixels(item['points']);color=m['color']
        ax.imshow(image);ax.plot(p[:,0],p[:,1],color=color,lw=1.4,alpha=.4)
        ax.plot(q[:,0],q[:,1],color=color,lw=1.8,ls='--')
        ax.scatter(q[:,0],q[:,1],s=34,color=color,edgecolor='white',zorder=4)
        for i,point in enumerate(q,1):
            ax.annotate(str(i),point,xytext=(6,-10),textcoords='offset points',fontsize=10,
                        color=color,bbox=dict(facecolor='white',alpha=.8,edgecolor='none',pad=.5))
        axall.plot(q[:,0],q[:,1],color=color,lw=1.7,ls='--',alpha=.85,
                   marker='o',markersize=4,label=f'S{number} · {m["waypoints"]}점 / 약 {m["reference_length_m"]:.0f}m')
        for target in (ax,axall):
            target.scatter(*p[0],s=65,marker='o',color=color,edgecolor='white',zorder=5)
            target.scatter(*p[-1],s=110,marker='*',color=color,edgecolor='white',zorder=5)
            for idx in np.linspace(100,len(p)-100,5).astype(int):
                target.annotate('',xy=p[idx+12],xytext=p[idx-12],arrowprops=dict(arrowstyle='->',color=color,lw=2))
        ax.set_title(f'S{number} · {m["label"]}\n약 {m["reference_length_m"]:.0f}m / 웨이포인트 {m["waypoints"]}개 / 약 {m["nominal_driving_sec_at_6mps"]:.0f}초 (6m/s)',fontsize=13)
        ax.axis('off')
    fig.suptitle('추가 장거리 시나리오 S22~S25 · SDMap 수준의 성긴 웨이포인트',fontsize=18)
    fig.supxlabel('번호 있는 점·점선: 실제 웨이포인트와 연결선 / 옅은 실선: 도로를 따르는 참고 경로 / ★ 종료 / 실제 주행 자취 아님',fontsize=10)
    fig.savefig(output/'long_scenarios.png',dpi=150)
    axall.legend(loc='upper left',fontsize=12);axall.set_title('S22~S25 장거리 시나리오 전체 배치 · ● 출발 / ★ 종료',fontsize=16)
    overview.savefig(output/'long_scenarios_overview.png',dpi=150)
    report=dict(source_reference=str(reference),source_reference_sha256=hashlib.sha256(reference.read_bytes()).hexdigest(),
                written=args.write,scenarios=[i['metadata'] for i in result.values()])
    (output/'design_report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    for item in result.values():print(json.dumps(item['metadata'],ensure_ascii=False),flush=True)


if __name__=='__main__':main()
