# KNU 경로와 전체 시나리오 자동 실행

scenario1–21을 지원한다. **전체 자동 실행은 [BATCH_RUN.md](BATCH_RUN.md)** 참고.
scenario5–21의 좌표는 첨부 `knu_section_routes.m`에서 순서와 값을 그대로 옮겼다.
시작 위치·방향 정렬도 21개 모두 지원한다. 기본 단일 경로 실행은 기존 동작을 유지한다.

아래 내용은 scenario1–4를 처음 분리했던 기록이다. 현재 지원 범위는 위의 21개다.

# KNU 전역 경로

`scenario1.csv`부터 `scenario4.csv`까지는 제공된 `get_navigation_waypoints.m`의
`case {"knu","osm"}` waypoint 배열을 좌표와 순서 변경 없이 옮긴 데이터다.
각 행은 `map` 좌표계의 `x,y,z`이며 단위는 m다.

`map_gen_knu.m`은 도로를 만든 뒤 해당 함수의 waypoint 배열을 반환한다.
따라서 전역 경로를 공급하는 데 MATLAB, `processed_road_map.mat`, 도로 폭 데이터,
`drivingScenario`는 필요하지 않다. 도로 형상에서 경로를 새로 탐색하거나
waypoint 간격을 변경하는 처리는 수행하지 않는다.

scenario2의 첫 waypoint `(-250, 101)`은 원본 그대로다.
차량 초기 배치점 `(-252.789, 93.548)`과 구별하며, 이번 변경에서는 기존
PX4 map 정렬 및 플래너 waypoint bias를 수정하지 않는다.

## 실행

워크스페이스에서 빌드하고 환경을 불러온 뒤 기존 split launch를 사용한다.

```bash
cd /home/imac/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select virtual_control
source /home/imac/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=1
ros2 launch virtual_control tracking_control_split.launch.py route_name:=scenario3
```

제공된 `main.m`은 scenario3을 선택하고 있다. 기존 launch 기본값은 scenario1을
유지했으므로 위와 같이 원하는 경로를 명시한다. `route_name`은 새 전역 경로 노드와
기존 PX4 map bridge에 함께 전달된다. MATLAB에서 생성하는 도로/grid의 시나리오도
동일하게 선택한다.

전역 경로만 발행하려면 다음과 같이 실행한다.

```bash
ros2 run virtual_control knu_global_path_publisher.py --ros-args -p route_name:=scenario3
```

기본 경로 토픽은 `/navigation/global_path` (`nav_msgs/Path`, `map`)다.
MATLAB의 `/debug/global_path`와 분리하여 MATLAB의 초기 발행이 ROS가 선택한
미션을 덮어쓰지 않게 한다. 기존 MATLAB 스크립트는 그대로 grid 생성에 사용할 수 있다.
`/grid_map` 생성은 계속 MATLAB에 의존하며, 이 변경은 전역 경로 생성·공급만 분리한다.

발행자와 상위 플래너 구독자는 reliable + transient-local QoS를 사용한다.
발행 노드가 살아 있으면 플래너가 늦게 시작하거나 재시작해도 저장된 최신 경로를 받는다.
2초마다 같은 경로를 다시 발행하며, 동일 경로 수신 시 waypoint 진행을 유지하는
기존 플래너 동작을 사용한다. 발행자 재시작 시에는 선택한 파일을 다시 읽어 발행한다.

`publish_period_sec:=0.0`은 최초 1회 발행 후 저장만 유지하는 설정이다.
토픽을 바꾸려면 launch의 `global_path_topic` 인자를 사용한다. 발행자와 플래너에
동시에 적용된다. MATLAB이 계속 발행하는 `/debug/global_path`를 새 발행자의 토픽으로
지정하지 않는다.

기존 MATLAB 경로 입력으로 실행해야 할 경우에는 아래처럼 ROS 경로 발행자를 끈다.
MATLAB 발행자는 현재 코드처럼 transient-local이어야 한다.

```bash
ros2 launch virtual_control tracking_control_split.launch.py \
  enable_global_path_publisher:=false global_path_topic:=/debug/global_path \
  route_name:=scenario3
```

## 이번 변경 범위

- QP 실패 시 이전 조향 시퀀스를 소비하는 하위 제어 코드를 유지한다.
- `solver_hold_last_valid_sec=0`은 시퀀스 소진 후 hold를 하지 않는 기존 설정이다.
- 좌표 보정, 제어 주기, horizon, 속도, 조향 모델 파라미터를 유지한다.

## 검증 기록 (2026-09-10)

- `virtual_control` 증분 빌드 성공. 기존 Eigen NEON 헤더의 컴파일 경고는 남아 있다.
- 원본 MATLAB 함수와 4개 시나리오의 28개 waypoint 전체 좌표·순서 일치.
- 경로 검증 pytest 18개 및 기존 C++ 핵심 테스트 26개 통과.
- 새 Python 코드의 flake8 및 구문 검사, `git diff --check` 통과.
- 별도 ROS 도메인에서 1회만 발행한 경로를 늦게 생성한 구독자와 재생성한 구독자가 수신.
- 실제 `upper_planner_node`의 최초 시작 및 재시작에서 저장된 경로 수신.
- split launch의 scenario4 및 사용자 지정 토픽 선택, 반복 경로 발행 시 미션 재로딩 없음 확인.

통신 시험에서는 PX4 연결과 하위 제어를 실행하지 않았다. MATLAB grid 생성과
차량 주행을 포함한 전체 미션 완료는 이번 검증 범위에 포함하지 않는다.
