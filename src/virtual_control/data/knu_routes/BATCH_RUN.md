# KNU SIH 전체 시나리오 자동 실행

`scenario1`부터 `scenario21`까지 한 번씩 실행한다. 성공 또는 실패를 기록한 즉시 다음
경로 준비로 넘어간다. PX4 SIH는 계속 실행하며, 정지 확인 후 현재 PX4 로컬 위치를
다음 경로 시작점으로 **맵 좌표에서 재정렬**한다. PX4를 재부팅하거나 물리 위치를
텔레포트하지 않는다. 각 경로의 제어 상태와 상위 waypoint 진행은 초기화한다.
이 전환은 SIH에서 사용하는 가상 맵 기준이다.

## 기존 맵 공급기를 쓰는 실행

기존 단일 경로 `tracking_control_split.launch.py`를 종료하고, PX4 SIH와 기존 맵
공급기는 계속 켜 둔다. 기존 PX4 연결·MANUAL 모드·arming 조건을 그대로 사용한다.
자동 arm 설정은 바꾸지 않았다. 현재 YAML은 맵을 `/bev/occupancy_grid`로 받는다.
맵 공급기는 `/motive/vehicle/odom_map` 또는 해당 정렬에서 나온 pose를 따라야 한다.

```bash
cd /home/imac/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select virtual_control
source /home/imac/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=1
ros2 launch virtual_control knu_batch.launch.py
```

배치 launch는 SIH용 MAVLink 출력을 활성화한다 (`mavlink_enable=true`,
`pixhawk_output_backend=mavlink_udp`). 기존 연결은 YAML의 USB bridge 및 UDP 14540
설정을 사용한다. 연결값을 별도 파일에서 관리한다면 `config_file:=/절대경로/설정.yaml`을
전달한다. PX4 raw pose bridge가 따로 실행 중이면 `enable_px4_bridge:=false`로 실행한다.
단일 경로 발행자·기존 map bridge·기존 upper/lower 제어기는 중복 실행하지 않는다.

MATLAB은 전역 경로 생성에 필요하지 않다. **도로/grid 공급기는 여전히 필요하다.**
현재 grid가 MATLAB에서 나오는 환경이면 다음 절의 실행을 사용한다.

출력 없는 연결 시험은 `mavlink_enable:=false pixhawk_output_backend:=disabled`를 덧붙인다.

## MATLAB이 `/grid_map`을 만드는 환경

워크스페이스 `matlab/knu_batch/`의 수정된 `main.m`과 함께 제공된 `.m` 파일들을 기존
MATLAB KNU 작업 폴더에 적용한다. MATLAB에서 `main`을 실행하고 ROS에서는 다음을 실행한다.

```bash
ros2 launch virtual_control knu_batch.launch.py grid_source:=matlab
```

`main.m`의 `enableKnuBatch=true`가 기본이다. 이 모드에서는 MATLAB이 전역 경로를
발행하지 않으며, `/knu_batch/active_route`를 받아 다음 경로 이름과 로그를 갱신한다.
KNU 전체 도로 형상은 같으므로 매번 맵을 다시 만들지 않는다. 정렬된 pose로 새 grid를
발행한 후 경로별 토큰을 확인 응답한다. ROS는 이 응답을 기다린 뒤 주행을 허용한다.
마지막 경로가 끝나면 MATLAB 루프도 끝난다.

기존 작업 폴더의 `processed_road_map.mat`, `experiment_scale_profile.m`,
`camera_module.m`, `update_vehicle_wheels.m` 및 그 의존 함수는 계속 필요하다.
이번 첨부에는 이 파일들과 MATLAB 실행 환경이 없으므로 새 폴더만으로 MATLAB을
독립 실행할 수는 없다. `width_samples.csv`는 기존 파일이 있으면 함께 유지한다.

## 기본 판정

| 조건 | 처리 |
| --- | --- |
| 도착점 8m 이내 + 진행률 85% 이상 + 남은 경로 12m 이하 | 완주, 다음 루트 |
| 위치가 0.5m 이상 바뀌지 않는 상태 10초 | `stopped_10s`, 다음 루트 |
| 현재 경로 구간에서 15m 이상 벗어난 상태 3초 | `off_route`, 다음 루트 |
| 최대 진행 위치보다 8m 이상 뒤로 간 상태 3초 | `wrong_direction`, 다음 루트 |
| 움직이지만 30초 동안 경로 진행이 없음 | `no_progress`, 다음 루트 |
| 위치 또는 grid 수신 중단 10초 | `pose_timeout` / `grid_timeout`, 다음 루트 |
| 갑작스러운 비정상 위치 점프 | `pose_jump`, 다음 루트 |
| 한 경로 주행 15분 초과 | `route_timeout`, 다음 루트 |
| 정지 확인 10초, 정렬·맵·계획 준비 각 30초 초과 | 준비 실패 기록, 다음 루트 |

완주에는 정지나 최종 방향 일치를 요구하지 않는다. 순환 경로·교차점에서 처음부터
도착점에 가깝다는 이유만으로 완주하지 않도록 경로 진행을 함께 확인한다.
다음 루트를 고른 뒤 정지 확인 1초, 새 맵 수신 확인, 새 계획 수신 확인을 수행한다.
실패를 완주로 표시하지 않는다. 설정은 `config/knu_batch.yaml`에서 변경한다.
상위 플래너 자체의 거리만 보는 완주 판정은 배치에서 끄고, 배치 실행기가 판정한다.

입력이 신선하지 않으면 전환 판정 10초를 기다리는 동안에도 주행 허용을 내리지 않는다.
배치 실행기가 사라지면 하위 제어기는 허용 신호 만료 후 1초 안에 중립 출력을 적용한다.
모든 경로가 끝나도 ROS 배치 노드는 정지 출력을 유지하도록 남는다. 창을 종료하면 된다.

## 결과와 일부 구간 재실행

실행한 디렉터리의 `knu_batch_results/날짜_세션/summary.csv`에 각 루트의 결과,
소요 시간, 진행률, 경로 이탈 거리, 도착점 거리와 실패 상세가 저장된다.
매 루트 종료마다 파일을 원자적으로 갱신하므로 중간 종료에도 앞선 결과는 남는다.
`settings.json`에 사용한 기준도 저장한다. MATLAB의 기존 pose/GIF 로그는 유지하며
`batch_routes.csv`에 경로 전환 시간과 토큰을 추가한다.

```bash
ros2 launch virtual_control knu_batch.launch.py start_scenario:=5 end_scenario:=21
```

`results_directory:=/절대경로`로 결과 폴더를 바꿀 수 있다.
현재 상태는 `/knu_batch/status`에 경로 번호, 총 경로 수, 단계, 결과 폴더로 발행한다.

## 검증

21개 경로 좌표 보존, 전 경로 정상 진행, 정지·이탈·역주행·점프·제한시간·순환 경로
오판 방지 테스트와 별도 ROS 도메인에서 실행하는 연결 시험을 포함한다.
실제 SIH 전체 주행은 PX4 및 맵 공급기가 연결된 사용자 환경에서 실행해야 한다.
