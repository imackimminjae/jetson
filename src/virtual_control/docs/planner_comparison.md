# MIQP / NVP / MPCC 비교 구현

작업 대상은 Windows `C:\Users\rlaal\EGO_MIQP_MPC\hils`에 해당하는 Linux ROS 2
workspace `/home/imac/ros2_ws`이다. 이 작업에서 적용할 `AGENTS.md`는 발견되지 않았다.
작업 전부터 존재한 MIQP, 하위 MPC, 설정 및 시나리오 수정은 보존한다.

## 빌드와 실행

ROS 2 Humble, C++17, Eigen3, 기존 `mpclib_vendor`가 필요하다. MPCC는 vendor가
이미 빌드·설치·export하는 `mpclib_vendor::daqp`를 직접 사용한다. 새 솔버 다운로드,
MATLAB, Python 최적화 패키지는 필요 없다. 패키지 의존성에 Eigen을 명시했다.
[DAQP 공식 설명](https://darnstrom.github.io/daqp/)에 명시된 dense convex QP 솔버를
사용하며, SQP 각 회차의 목적함수 Hessian에 작은 양의 정규화를 둔다.

```bash
cd /home/imac/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build --packages-up-to virtual_control \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
source install/setup.bash

# 기존 MIQP와 동일한 시뮬레이션 연결
ros2 launch virtual_control planner_comparison.launch.py planner:=miqp
ros2 launch virtual_control planner_comparison.launch.py planner:=nvp
ros2 launch virtual_control planner_comparison.launch.py planner:=mpcc reference_source:=sparse

# 정확한 기준선은 별도 publisher / rosbag으로 공급해야 한다.
ros2 launch virtual_control planner_comparison.launch.py planner:=mpcc \
  reference_source:=exact exact_reference_topic:=/navigation/exact_reference_path

colcon test --packages-select virtual_control \
  --ctest-args -R 'test_comparison_(planners|adapter)' --output-on-failure
colcon test-result --verbose
```

비교 선택은 새 `planner_comparison.launch.py`에서만 제공한다. 기존
`tracking_control_split.launch.py`와 기존 상위 MIQP 소스/헤더는 이번 작업에서 변경하지 않는다.
새 실행 파일은 `nvp_planner_node`, `mpcc_planner_node`이다. 기본은 `planner:=miqp`이다.
새 launch는 기존 launch를 include하면서 NVP/MPCC 선택 시 기존 `enable_upper_planner`를
false로 전달하고 별도 노드 하나를 시작한다. 따라서 선택한 상위 노드와 기존 하위 MPC만 동작한다.
다른 상위 노드가 실행 중인 상태에 중복 launch하지 않는다. `enable_px4_bridge`,
`enable_px4_odom_map_bridge`, `enable_global_path_publisher`, `route_name`, `config_file`,
토픽 override 등 기존 인자가 유지된다. 현재 비교 launch는 fullscale + Odometry
입력을 지원한다. 모델 축척이나 PoseStamped 전용 설정은 오류로 중단한다.

`comparison_config_file`로 `config/comparison_planners.yaml`의 복사본을 지정하면
플래너별 가중치·검증 여유·초기 진행도를 바꿀 수 있다. 실행 중 동적 파라미터 변경은
지원하지 않으며 변경 후 재시작한다. 정확한 기준선의 출처, 측량/설계 방법, 곡선화 오차를
실험 기록에 별도로 남긴다. `enable_waypoint_bias=false`라는 이유만으로 정확한 기준선으로
판정하지 않는다. exact 모드에서는 waypoint bias를 켤 수 없다.

기존 batch runner는 `/upper_planner_node/reset_mission`을 고정 호출한다. 기존 코드를
변경하지 않았으므로 비교 노드의 반복 실험에서는 `/controller/reset`을 함께 발행하거나
`/nvp_planner_node/reset_mission`, `/mpcc_planner_node/reset_mission`을 호출하는 별도
실험 구동부가 필요하다. 기존 batch runner와의 자동 반복 완주 검증은 이번 범위에 포함하지 않는다.

## 공통 연결과 비교 조건

| 항목 | 현재 C++ 조건 / 동작 |
|---|---|
| 목표/고정 예측 속도 | 6.0 m/s. 기존 MIQP도 `planning_speed_mps=target_speed_mps_` 사용 |
| 상위 주기 / 예측 | 5 Hz / 0.5 s × 7 = 3.5 s, 명목 21 m. YAML의 오래된 0.75 s/31.5 m 주석은 실제 값과 다름 |
| 하위 추종 | 기존 `lower_tracking_mpc_node`, 10 Hz, 0.1 s × 30, 실측 속도 기반 진행 |
| 기하 | wheelbase 2.8 m, 길이 4.7 m, 추종점에서 후륜까지 1.4 m |
| 조향 | 각도 0.30 rad, 하위 조향 속도 1.0471975512 rad/s |
| 폭 가정 | 비교군은 명시적으로 1.8 m. 기존 MIQP는 별도 폭 파라미터가 없음 |
| 여유 | 반폭 0.9 + extra 1.1 = 2.0 m, 기존 interval boundary margin과 명목 횡방향 여유 일치. 격자 샘플 누락 방지용 반 셀 대각선 패딩 추가 |
| 상태 | `/motive/vehicle/odom_map`, frame `map`, 현재 설정은 `orientation.z`가 yaw scalar. 실측 속도는 twist x/y의 크기 |
| BEV | `/bev/occupancy_grid`, frame `Vehicle`, 기본 0=주행 가능, 100=비주행, -1=미관측 |
| 의도 경로 | `/navigation/global_path` (`nav_msgs/Path`), frame `map` |
| 정확한 MPCC 기준 | `/navigation/exact_reference_path` (`nav_msgs/Path`), frame `map`, 외부 공급 |
| 출력 | `/planner/upper_path_sparse`, `/planner/lower_reference_path`, `/planner/lower_reference_path/with_arclength` |
| 정지/리셋 | 기존 `/planner/goal_reached`, `/controller/reset`; 비교 노드 `~/reset_mission` Trigger도 제공 |

속도·예측 주기·기본 토픽·극성은 split YAML의 상위 노드 설정을 읽어 적용한다.
하위 MPC 설정의 조향 속도 및 추종점 오프셋도 재사용한다. 비교 전용 설정은
알고리즘 가중치와 정책만 담는다. 플래너는 조향·스로틀·차량 명령을 발행하지 않는다.
실제 차량 출력, 속도 PI, 조향 액추에이터 모델은 하위 노드가 계속 담당한다.

코어 입력에는 실제 속도와 직전 적용 조향이 있다. MPCC의 고정 예측 속도는 목표
속도이며 actual speed를 덮어쓰지 않는다. 가상 진행 속도 `v_s`는 별도 최적화 변수이다.
따라서 출발·감속 시 실제 차량과 상위 예측 사이의 속도 불일치가 존재한다. 이는 기존
MIQP의 고정 목표 속도 조건을 맞춘 실험 선택이다. 실제 속도에 인위적 하한을 적용하지 않는다.
`/debug/lower_mpc_trace` index 20의 마지막 적용 조향을 신선할 때 사용한다. 이 값은
센서 측정 조향이 아니다. 유효한 피드백이 없으면 최초 조향 변화 제약의 anchor는 0이고
진단에 `applied_steer_available=false`를 남긴다. index 19의 액추에이터 추정값과 구분한다.

경로는 기존 `densifyPath`와 같은 선형 호장 보간 규칙을 사용한다. 간격은
`max(target_speed*lower_dt, min_spacing)` = 0.6 m이며 끝점을 포함한다. 원래
polyline의 호장을 보존하여 같은 계산 결과에서 좌표와 `s`를 만든다. 조밀화로 꼭짓점이
생략될 수 있으므로 최종 발행 선분도 별도로 충돌 검사한다. `prepareLowerPath`로
크기·유한성·호장 단조성을 확인한다. 최적화 평활화는 추가하지 않았다.

종점까지 남은 거리가 일반 최소 길이보다 작아도 goal 반경 바깥에서 멈춰버리지 않도록,
끝점이 실제로 가까운 마지막 접근에서만 최소 길이를 남은 거리의 절반으로 줄인다.
NVP는 마지막 두 원의 간격도 줄인다. 그 외의 짧은 경로는 실패로 반환한다.

## 좌표계, QoS, 시간, 실패 정책

* 경로 입출력과 goal은 기존과 같은 reliable/transient-local/depth 1이다.
  Odometry/BEV는 기존의 reliable depth 10, 조향 trace는 best-effort depth 1이다.
* `map` BEV와 `Vehicle` BEV만 수용한다. 임의 TF 추측은 하지 않는다. 회전된
  OccupancyGrid 원점을 정확히 역변환하고 음수 셀 좌표는 floor로 처리한다.
  격자 원점 x 강제 변경 실험은 비교 launch에서 거부한다.
  매 스냅샷마다 known-free 누적 격자를 만들어 완전히 빈 bounding box는 빠르게 통과시키며,
  장애물/unknown/지도 경계가 섞인 경우 원래 회전 footprint 검사를 수행한다.
* `Vehicle` BEV는 수신 당시/획득 timestamp에 해당하는 Odometry 이력을 보간해
  월드에 고정한다. 계획 시점 ego 위치로 과거 지도를 재부착하지 않는다. bracket 밖에서는
  가장 가까운 pose가 0.1 s 이내일 때만 허용한다. 보간 이력 간격은 0.2 s 이하이다.
* 상태/지도는 수신 시각과 유효한 header timestamp 모두 0.6 s 이내여야 한다.
  0 timestamp는 수신 시각으로 대체한다. 미래 0.05 s 초과, 역순 상태, 다른 frame,
  비정상 quaternion/격자/경로는 거부한다. yaw scalar 해석은 Odometry에만 적용한다.
  ROS clock 역행, 위치 점프, 변경된 경로, reset은 플래너 해와 진행 이력을 초기화한다.
  동일한 경로 heartbeat는 진행도/warm start를 초기화하지 않는다.
* 알려진 비주행 셀은 항상 차단한다. 미관측/지도 밖도 차단한다. 전방 전용 BEV를 위한
  `allow_initial_unobserved_footprint=true` 예외는 **현재 차량이 이미 차지하는 padded
  rectangle 안**의 unknown/outside 샘플에만 적용한다. 새로 진입하는 영역에는 적용하지 않는다.
  false로 바꾸면 rear footprint가 지도 밖인 전방 전용 지도에서 시작부터 실패할 수 있다.
* 연결 검사에는 길이·폭·여유를 포함한 rectangle을 사용하며 이동 및 회전을 반 셀 이하로
  샘플한다. 셀 대각선 패딩은 보수적이다. 도로 중심점만 검사하는 구현과 다르다.
* 비교군은 실패마다 빈 Path/PathWithArcLength를 발행해 하위의 기존 invalid-input
  neutral 정책을 즉시 실행한다. 이전 최적 경로를 성공처럼 재발행하지 않는다. 기존 MIQP의
  stale/missing 지도 시 마지막 경로 유지 정책은 변경하지 않았다. 따라서 이 실패 정책 차이와
  하위의 1.5 s path timeout도 실험 결과에 함께 기록해야 한다.
* 도착은 열린 경로의 진행도와 실제 끝점 거리 모두 0.5 m 이내일 때만 선언한다.
  실패와 도착은 진단에서 구분된다. goal은 reset/새 경로까지 유지된다.

기존 MIQP의 목적함수, branch guidance, soft corridor 및 제약은 수정하지 않았다.
MIQP의 nominal fallback/soft constraint와 비교군의 보수적 unknown/footprint 정책은
완전히 같은 실행 가능 집합을 보장하지 않는다. 동일 하위 추종기를 사용한 **상위 경로계획
구현 비교**이며 이 차이를 숨긴 알고리즘 우열 판단에 사용하면 안 된다.

## NVP local planner + common tracking MPC

참고: [NVP 논문, IV-B/C](https://arxiv.org/abs/2108.09117),
[저자 프로젝트](https://aurova.ua.es/osm_path_planning/).

제공 waypoint 중 전방 local goal을 선택하고 BEV로 최근접 장애물 Euclidean distance
field를 계산한다. 각 전방 동심원에서
`w_r / d_obstacle^gamma_r - w_a / d_goal^gamma_a`를 샘플한다. 원호 방향 gradient가
작은 연결 영역마다 가장 낮은 potential 후보를 남긴다. 가장 바깥 원에서 goal에 가까운
후보를 택한 뒤, 안쪽 원마다 연결 가능한 가장 가까운 후보로 잇는다. 마지막으로 ego 시작점을
붙이고 전방 순으로 뒤집는다. 비용의 거리 특이점은 해상도로 제한한다.

원 알고리즘의 naive 연결에 선분/footprint 충돌 검증과 최대 연결 길이, 전방 연속성 검사를
추가했다. 최단 후보가 충돌하면 같은 원의 다음 후보를 시도하되 A*/전역 backtracking으로
대체하지 않는다. 원에 후보가 없거나 안쪽 연결이 막히면 명시적으로 실패한다. NVP 경로는
차량 운동학 최적해가 아니며 하위 MPC의 추종 가능성을 보장하지 않는다.

이번 구현은 논문의 전체 시스템 재현이 아니다. LiDAR ground segmentation/free-space
polygon 전처리, OSM 그래프 탐색, GPS/IMU 융합, 원 논문의 제어 명령 선택 및 속도
법칙은 구현하지 않는다. 이미 생성된 BEV와 waypoint를 사용하고 제어부를 기존 tracking
MPC로 대체한다. 논문상의 통합 시스템 성능/계산시간 수치를 이번 구현의 성능으로 인용하지 않는다.

## 고정 속도 기구학 MPCC

참고: [원 프로젝트](https://github.com/alexliniger/MPCC),
[공개 C++ 상태/입력 정의](https://github.com/alexliniger/MPCC/blob/master/C%2B%2B/types.h).

이번 상태는 `(X,Y,psi,s)`, 입력은 `(delta,v_s)`이다. 고정 실제 운동 예측 속도 `v`와
`v_s`를 구분한다. `beta=atan((reference_to_rear/wheelbase)*tan(delta))`이며 전진 Euler:

```text
X+ = X + dt*v*cos(psi+beta)
Y+ = Y + dt*v*sin(psi+beta)
psi+ = psi + dt*v*cos(beta)*tan(delta)/wheelbase
s+ = s + dt*v_s
e_c = normal(s) dot ((X,Y)-r(s))
e_l = tangent(s) dot ((X,Y)-r(s))
cost = sum(q_c*e_c² + q_l*e_l² + r_delta*delta²
           + r_change*((delta-delta_previous)/interval)²) - reward*s_terminal
```

SQP에서 운동을 선형화하고 상태를 소거한 dense QP를 DAQP로 푼다. contour/lag의
진행도 미분에 접선·곡률을 포함한다. 초기 상태는 측정 상태로 고정한다. 조향/조향 속도,
`0<=v_s<=9`, `0<=s<=curve_length`, 진행도/조향 trust region, 도로 corridor를 hard
constraint로 적용한다. 최초 조향 변화는 직전 적용 조향에 계획 주기 0.2 s로 연결한다.
솔버 성공 코드만 믿지 않고 QP residual, nonlinear rollout의 조향/진행/corridor,
출력 선분 충돌을 다시 검사한다. 실패 시 해를 발행하지 않는다. 제한된 SQP 회수에 의한
국소해이며 전역 최적성이나 nonlinear KKT 수렴을 보증하지 않는다.

기준 곡선은 open cubic Hermite와 수치 호장 lookup으로 위치·접선·곡률·투영을 제공한다.
exact 모드도 외부 sample을 곡선화한 근사이며 원래 연속 곡선과 동일하다고 보장하지 않는다.
sparse 모드는 같은 보간기를 사용하되 자료 출처를 별도 진단으로 남긴다. `s` 투영은 이전
측정 진행도에서 후방 1 m, 전방 `actual_speed*elapsed+2 m` 창 안에서만 수행한다.
기본 최초 진행도 seed는 0이다. 중간에서 시작하면 seed를 명시한다. -1은 최초에만 전역
투영을 허용하므로 가까운 반복 구간에서 모호할 수 있다. 이전 입력열은 경과시간을 이용해
보간하고 현재 초기 상태에서 다시 rollout한다. 0.2 s마다 무조건 0.5 s 한 칸 이동하지 않는다.

각 normal section에서 기준점이 속한 단 하나의 연속 free interval만 corridor로 만든다.
중간 비주행 셀을 건너 다른 branch의 interval까지 합친 envelope는 사용하지 않는다.
기준 자체가 장애물/비주행에 있으면 해당 계획은 실패한다. MIQP가 선택한 branch/최적해를
구독하지 않는다. 별도 obstacle homotopy search나 다른 branch 탐색은 제공하지 않는다.
열린 경로 끝에서는 예측 길이/마지막 dt를 줄이고 진행도를 clamp하며 끝점 overshoot를
검사한다. 종점 부근에도 고정 속도로만 예측하므로 정지 최적화는 없고 goal 신호와 기존
하위 제어기가 정지를 담당한다.

공개 MATLAB의 duty/steering/virtual speed 정의를 C++와 동일하다고 가정하지 않았다.
공개 C++는 동적 bicycle, tire 모델, 추가 입력 상태 `(D,delta,v_s)`와 변화율 입력
`(dD,dDelta,dVs)`를 사용한다. 이번 코드는 해당 소스를 연결한 것이 아니라 목적과 진행도
구조를 참고해 작성한 기구학 상위 플래너다. 원 코드의 HPIPM 대신 기존 DAQP를 사용하며
동적 tire/drive train/속도 최적화는 없다. 원 저장소가 설명하는 MATLAB 장애물 회피 모듈을
공개 C++의 기본 기능으로 가정하지 않았다. BEV 기반 corridor/충돌 검사는 이번 비교 구현이다.

## 진단과 측정

`/debug/comparison_planner_status`는 JSON String으로 성공, 실패 이유, 입력 종류,
실측/예측 속도, 적용 조향 사용 여부, 투영 진행도, warm shift, QP 상태/회수,
residual, 후보 수 및 시간을 기록한다. `/debug/comparison_planner_timing` 배열은
`[ROS stamp, preprocessing_ms, planning_ms, postprocessing_ms, total_ms, success]`이다.

시간은 steady clock wall time이다. 전처리는 adapter의 입력 확인/변환과 reference 및 NVP
distance field 준비, planning은 NVP 후보/연결 또는 MPCC SQP와 nonlinear 검증, 후처리는
조밀화/최종 geometry 검증/ROS 경로 메시지 생성·발행을 포함한다. 전체는 timer callback에서
상태 진단 메시지 자체를 내보내기 직전까지다. 수신 callback/DDS 대기 시간과 별도 status
메시지 생성·발행 비용은 포함하지 않는다. MIQP 기존 stage timing은 그대로 유지된다.
시뮬레이션에서 동일한 구간을 rosbag으로 기록하여 계획 시간뿐 아니라 실패율, 추종 오차,
조향, 실제 속도, 완료율을 함께 평가한다.

## 변경 파일

* `include/virtual_control/comparison_planner.hpp`: 공통 ROS 독립 API.
* `src/comparison_geometry.cpp`, `src/comparison_internal.hpp`: 지도/곡선/충돌/호장/공통 검증.
* `src/nvp_planner.cpp`, `src/mpcc_planner.cpp`: 독립 플래너와 이전 해 상태.
* `src/comparison_planner_node.cpp`, `src/comparison_planner_main.cpp`,
  `include/virtual_control/comparison_planner_ros.hpp`: 새 NVP/MPCC 노드의 공통 ROS 어댑터,
  시각/좌표계 검증, 진단, reset. 기존 상위 노드와 별도 실행 파일이다.
* `config/comparison_planners.yaml`, `launch/planner_comparison.launch.py`: 비교 설정/진입점.
* 기존 `launch/tracking_control_split.launch.py`: 변경 없음. 새 launch에서 기존 옵션으로 재사용.
* `CMakeLists.txt`, `package.xml`: 코어/노드/테스트 타겟 및 DAQP/Eigen 의존성.
* `test/test_comparison_planners.cpp`, `test/test_comparison_adapter.cpp`,
  `test/test_comparison_launch.py`, `test/test_comparison_pipeline.py`: 코어/ROS/선택/하위 연동 검증.

검증 실행 결과와 미실행 범위는 `planner_comparison_validation.md`에 기록한다.
