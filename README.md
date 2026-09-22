# PX4 / Isaac HILS 제어

ROS 2 Humble 기반 HILS(Hardware-in-the-Loop Simulation) 워크스페이스다.
PX4 상태를 KNU 맵 좌표로 변환하고, OccupancyGrid 기반 상위 MIQP 경로 계획과
하위 MPC 추종 제어를 실행한다.

| 경로 | 내용 |
| --- | --- |
| `src/virtual_control` | PX4 브리지, 상·하위 제어기, launch·설정, KNU 21개 경로 및 배치 실행·테스트 |
| `src/imac_interfaces` | `VirtualControlCommand` ROS 메시지 |
| `src/mpclib_vendor` | 제어기에 필요한 MIQP/QP·DAQP 솔버 소스와 원본 라이선스 |
| `matlab/knu_batch` | 선택적 MATLAB grid 공급·배치 연동 및 경로 검증 코드 |

구형 제어 코드, 로컬 백업, 빌드 결과, rosbag 및 주행 로그는 업로드 대상에서 제외한다.

## 빌드

ROS 2 Humble 환경에서 저장소 루트로 이동한 뒤 실행한다.
ROS 의존성은 각 패키지의 `package.xml`에 선언되어 있다. PX4 Python 브리지는
`pymavlink`가 필요하다. MIQPLib의 선택 기능에는 GSL, libconfig++, libmatheval이 사용된다.

```bash
git clone https://github.com/imackimminjae/jetson.git
cd jetson
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to virtual_control --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## 실행

PX4 연결과 Isaac BEV 공급기(`/bev/occupancy_grid`)는 별도로 실행한다.

```bash
export ROS_DOMAIN_ID=1
ros2 launch virtual_control tracking_control_split.launch.py route_name:=scenario3
```

위 단일 경로 실행은 MAVLink 출력이 기본 비활성화되어 있다. 기존 HILS의 하드웨어
출력을 사용하려면 `mavlink_enable:=true pixhawk_output_backend:=mavlink_udp`를 추가한다.
장치·연결 설정은 `src/virtual_control/config/tracking_control_split.yaml`에서 확인한다.

```bash
# 배치 실행은 MAVLink 출력이 기본 활성화된다.
ros2 launch virtual_control knu_batch.launch.py
```

MATLAB 연동은 기존 MATLAB 작업 폴더의 외부 의존 파일이 추가로 필요하다.
상세 구성은 [HILS 안내](README_HILS.md), 경로·실행 조건은
[KNU 경로 안내](src/virtual_control/data/knu_routes/README.md)와
[배치 실행 안내](src/virtual_control/data/knu_routes/BATCH_RUN.md)를 참고한다.

## 테스트

```bash
ctest --test-dir build/virtual_control --output-on-failure -R '^test_(px4_odom_map_transform|egocentric_planner_geometry|interval_centering_state|lower_steering_actuator_model|miqp_constraint_enforcement)$'
python3 -m pytest -q src/virtual_control/test
```

ROS 통합 테스트는 기본적으로 건너뛴다. 테스트 파일의 opt-in 설정을 적용하면 별도
ROS 도메인에서 합성 입력으로 실행할 수 있다. 실제 PX4/Isaac 주행 검증은 별도로 수행한다.
