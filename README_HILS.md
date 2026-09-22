# 현재 HILS 코드 구성

2026-09-22 기준 `tracking_control_split.launch.py`와 `knu_batch.launch.py`의
PX4/Isaac BEV/KNU 상·하위 제어 흐름을 기준으로 정리했다.

| 경로 | 역할 |
| --- | --- |
| `src/virtual_control` | PX4 상태 수신, 맵 좌표 변환, 상위 MIQP, 하위 MPC, KNU 경로·배치 실행 |
| `src/imac_interfaces` | 하위 제어기가 발행하는 `VirtualControlCommand` 메시지 |
| `src/mpclib_vendor` | MIQP/QP 솔버와 DAQP 의존 코드 |
| `matlab/knu_batch` | 배치 실행의 선택적 MATLAB grid 공급기와 경로 검증 자료 |

현재 C++ 실행 대상은 `px4_odom_map_bridge`, `upper_planner_node`,
`lower_tracking_mpc_node`이다. 상위 플래너 구현 파일명은 `tracking_control.cpp`다.
Python 실행 스크립트는 `px4_ekf_bridge.py`, `knu_global_path_publisher.py`,
`knu_batch_runner.py`, `knu_batch_monitor.py`를 유지한다.

관련 테스트·문서·KNU 시나리오 데이터도 유지한다. `local_map*.csv`는 기존 launch의
선택적 CSV 입력에서 참조하므로 유지한다. `hierarchical_control.launch.py`는 현재
split launch를 호출하는 진입점으로 유지한다.

## 빌드

```bash
# 이 저장소를 clone한 디렉터리에서 실행한다.
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to virtual_control --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

빌드 대상은 `imac_interfaces`, `mpclib_vendor`, `virtual_control` 3개 패키지다.
로컬에 보관한 `archive/`는 GitHub 업로드에서 제외하며, 기존 워크스페이스에서는
`archive/COLCON_IGNORE`로 빌드 탐색에서도 제외한다.

## 기존 HILS 실행

단일 시나리오의 기존 실행 방법과 파라미터는 유지한다.

```bash
export ROS_DOMAIN_ID=1
ros2 launch virtual_control tracking_control_split.launch.py route_name:=scenario3
```

기본 split 설정은 MAVLink 출력을 끈 상태다. 기존 HILS에서 사용하던 출력 설정을
적용하려면 `mavlink_enable:=true pixhawk_output_backend:=mavlink_udp`를 추가한다.
Isaac의 `/bev/occupancy_grid` 공급기와 PX4 환경은 별도로 실행되어 있어야 한다.
`config/isaac_bev_control.yaml`은 외부 BEV 공급기에 적용하는 설정이다.

배치 실행은 기존과 같이 다음 명령을 사용한다. 이 launch는 MAVLink 출력을 기본 활성화한다.

```bash
ros2 launch virtual_control knu_batch.launch.py
```

MATLAB grid 입력을 사용하는 배치는 `grid_source:=matlab`을 추가한다.
배치 설정과 외부 MATLAB 의존 파일은
[배치 실행 안내](src/virtual_control/data/knu_routes/BATCH_RUN.md)를 참고한다.

## 보관 자료

현재 실행 흐름 밖의 코드는 원래 작업 PC의 `archive/non_hils_20260922/`에 보관한다.
보관 기준과 복원 방법은 그 안의 `README.md`, 원래 경로는 `manifest.json`에 기록했다.
이 보관 폴더와 rosbag, `knu_batch_results`, 주행 로그, 빌드 산출물은 GitHub에 올리지 않는다.
로컬 파일은 기존 위치에 유지한다. GitHub에는 HILS 소스와 설정, 실행에 필요한 경로
데이터, 관련 테스트·문서 및 선택적 MATLAB 연동 코드가 포함된다.

## 정리 후 확인

2026-09-22: 남긴 3개 패키지를 이전 빌드 산출물 없이 빌드한 뒤 Release 설정으로
설치했다. 단일·배치 launch 인자 로딩, C++ 테스트 47개, Python 테스트 75개,
별도 ROS 도메인의 통합 테스트 3개가 통과했다. 통합 테스트는 합성 입력을 사용하며
MAVLink 하드웨어 출력을 비활성화했다. 실제 PX4/Isaac HILS 주행은 실행하지 않았다.
