from pathlib import Path
import json,numpy as np,hashlib,shutil,math
h=Path(__file__).resolve().parent;p=h/'20261006_164936';a=np.load(p/'first_grid.npy');meta=json.loads((p/'first_metadata.json').read_text());frames=json.loads((p/'frames.json').read_text())
def sample_line(origin_x):
 ref=np.array([13.498,-.237]);ang=np.radians(-1.01);vec=np.array([-np.sin(ang),np.cos(ang)]);origin=np.array([origin_x,-20.]);rel=ref-origin;lo=-math.inf;hi=math.inf
 for i in range(2):
  v=sorted([(0-rel[i])/vec[i],(40-rel[i])/vec[i]]);lo=max(lo,v[0]);hi=min(hi,v[1])
 lambdas=[lo+i*.1 for i in range(math.floor((hi-lo)/.1)+1)]
 if lambdas[-1]<hi-1e-10:lambdas.append(hi)
 result={'free':0,'blocked':0,'unknown':0,'outside':0}
 for lam in lambdas:
  cell=np.floor((ref+lam*vec-origin)/.3125).astype(int)
  if not (0<=cell[0]<128 and 0<=cell[1]<128):result['outside']+=1;continue
  value=a[cell[1],cell[0]];result['free' if 0<=value<=1 else 'unknown' if value<0 else 'blocked']+=1
 return result
rows,cols=np.where(a==0);analysis={'frame_count':len(frames),'first_unique_values_counts':{str(int(v)):int(c) for v,c in zip(*np.unique(a,return_counts=True))},'free_cell_count_range':[min(r['free_cells'] for r in frames),max(r['free_cells'] for r in frames)],'first_free_bbox_indices':[int(cols.min()),int(cols.max()),int(rows.min()),int(rows.max())],'free_forward_cell_edge_received_m':2.95+(int(cols.max())+1)*.3125,'free_forward_cell_edge_effective_origin0_m':(int(cols.max())+1)*.3125,'all_blocked_columns_from':int(cols.max())+1,'threshold0_equals_threshold1':bool(np.array_equal((a>=0)&(a<=0),(a>=0)&(a<=1))),'logged_k3_line_on_received_snapshot':{'effective_origin0':sample_line(0.),'received_origin2_95':sample_line(2.95)}}
(p/'analysis.json').write_text(json.dumps(analysis,indent=2)+'\n');print(json.dumps(analysis,indent=2))
attachment=Path('/home/imac/.codex/attachments/d9da3304-332d-4e75-93fe-a31c272c9a5b/붙여넣은 텍스트.txt');shutil.copy2(attachment,p/'user_log.txt')
(p/'RESULT.txt').write_text('''2026-10-06 조정 후 BEV / interval_diag 경고 확인

결론
수신은 현재 정상 확인했다. 67프레임/수신간격13.42초/약4.92Hz.
송신 노드 rgb_to_occupancy, ROS_DOMAIN_ID1, /bev/occupancy_grid.
실제 상위 경고의 직접 원인은 k3 부근 횡단면에 주행 가능한 셀이 없다는 것이다.
reason3=NoValidDrivableRun, line_free0 blocked401. k3 약13.5m에서 막혀 N=2로 축소.
이후 k4에서 막히는 시점에는 N=3. 일부 시점 QP 불가능은 남은 제약도 만족하지 못했다는 뜻.
로그만으로 남은 QP의 어떤 행이 불가능한지는 확정하지 않았다.

실제 수신 배열
128x128,0.3125m/cell,origin(2.95,-20),orientation identity.
-1=0개,0=4311개,100=12073개(첫 프레임). 67프레임의0개수4311..4312.
그리드 바이트 해시는2종이며 첫/마지막 프레임은 일치. 정지/고정영상 여부는 이 통계만으로 확정 불가.
0인 셀은 열0..34에만 있고 열35..127은 모두100. 횡방향 전체 폭에 가까운 근거리 띠 형태.
최대0셀 바깥 경계: 송신 좌표13.8875m / 수신 원점0 해석10.9375m.
따라서 unknown이0개라는 이유로40m까지 도로가 정상 관측됐다고 해석할 수 없다.

원점
로그 received_x=2.950 effective_x=0.000 shift_x=-2.950;
운영 grid_origin_x_override_enabled=true가 셀 배열은 그대로 둔 채 좌표만2.95m 이동시킴.
원점 정합은 별도로 확인해야 하지만 이 값만 원복해도 먼 도로가 새로 생기는 것은 아니다.
현재 관측한 원배열 자체에서 약13.9m보다 먼 모든 셀이100이다.

문턱/예측 길이
현재 배열은0/100뿐이어서 grid_value_threshold0과1은 완전히 같은 free mask.
threshold를100으로 올리거나 positive_is_drivable을 근거 없이 바꾸는 해결책을 쓰지 않았다.
이번 문제는 k7 한 단계의 문제가 아니라 k3부터 도로 후보가 사라지는 상태.
input_timeout_sec3.0은 최신 입력 허용시간이며 이 단면0개 문제를 해결하지 않는다.

읽기 전용으로 확인한 실제 송신 파라미터
ROI x=2.95..42.95 / y=-20..20,ground_z=-1.0,occupancy_rotate_ccw_90=false,
occupancy_invalid_as_unknown=true,invalid_as_nonroad=true,
near_fill_mode=free,near_free_distance=0,road_value1/nonroad_value0,
occupancy_grid_rviz_standard_values=true,road_gray_max85/white_line_gray_min180,
keep_largest_component=true,publish_bev_images=false,publish_resized=false.
전체값은 상위 폴더 publisher_params_20261006_165115.json.
ROI/투영평면/좌표변환/분할/미관측 마스크 중 무엇이 원인인지는 현재 송신 소스와
입력 영상·중간 BEV를 대조하지 않아 미확정. 과거 참고 소스와 현재 실행 코드의 동일성도 미확정.
사용자는 '유효 전방 거리40으로 나오게 바꿨다'고 설명했다.
크기/미관측 표시는 맞더라도 실제 도로 형상이 맞는지는 별개이며, 이번 배열은 앞쪽 띠만 free.

운영 코드/설정/프로세스/ARM/하드웨어 변경 없음.
진단은 구독과 파라미터 조회만 수행. 파라미터 설정 서비스는 호출하지 않았다.
received_grid_explained.png: 송신 좌표와 수신기가 사용하는 좌표에서 같은 배열 비교.
analysis.json: 수치, 로그 첫 k3 단면을 현재 배열에서 다시 샘플링한 결과.
이 재샘플링은 소수3자리 로그 좌표/각도와 후속 수신 프레임을 사용하므로
로그 당시 모든 입력을 완전 재현한 QP 시험으로 세지 않는다.
''')
files=[q for q in p.iterdir() if q.is_file() and q.name!='artifacts_sha256.json'];(p/'artifacts_sha256.json').write_text(json.dumps({q.name:hashlib.sha256(q.read_bytes()).hexdigest() for q in sorted(files)},indent=2)+'\n')
