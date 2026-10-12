# SIH 헤딩 오류와 입력 차단

2026-09-23 `drive_debug_20260923_214944` 기록에서 SIH 실제 yaw와 PX4 EKF yaw가
주행 전부터 약 96°, 주행 중 85–140° 어긋났다. ROS NED→ENU 변환 자체는 정상이었다.

## 실제 FC 확인 및 복구

PX4 1.16.1, SYS_HITL=2, SYS_AUTOSTART=1104. 비무장·정지 상태에서 점검했다.

- `sensor_mag`, `vehicle_magnetometer`, `estimator_sensor_bias` 모두 시뮬레이션
  자력계 device_id=197388을 사용했다. 실제 자력계 선택 오류는 확인되지 않았다.
- 복구 전 `cs_mag=False`, `cs_mag_hdg=False`, `cs_mag_heading_consistent=False`.
  자기장 innovation이 거부됐고 내부 mag_bias는 약 [-0.11148, -0.18688, 0.17155] G였다.
- 개별 `ekf2` 명령은 module locked 상태여서 복구되지 않았다. 비무장 SIH 확인 후
  보드를 재부팅했다. PX4 파라미터와 센서 캘리브레이션은 변경하지 않았다.
- 재부팅 후 세 플래그가 모두 True, `fused=True`, `innovation_rejected=False`로 복구.
  mag_bias는 약 [0.00392, -0.00104, -0.00208] G로 줄었다.
- 정지 상태 637개 ATTITUDE 샘플을 가장 가까운 HIL_STATE_QUATERNION 수신 시각과
  비교: 오차 중앙값 0.783°, 최대 절댓값 0.834°.
- 초기 고장을 유발한 조건은 아직 확정하지 않았다. 정지 상태 복구 검증이며,
  주행 중 재발 여부·6 m/s 경로 추종 성능은 재주행 기록으로 확인해야 한다.

## 재발 시 잘못된 상태 전달 차단

`px4_ekf_bridge.py`의 `sih_heading_guard_enabled`를 SIH 공유 설정에서 켰다.
일반 노드 기본값은 False이며 실제 차량/외부 Motive 사용 시 꺼야 한다.

- 동일 MAVLink 시스템·컴포넌트의 ATTITUDE와 HIL_STATE_QUATERNION을 NED에서 비교.
- 절댓값 15° 이내, 두 데이터 모두 0.25초 이내이고 2초 연속 정상일 때 pose 발행.
- 불일치, 누락, 오래된 데이터, 잘못된 quaternion에서는 pose/odom 발행을 중단.
  하위 제어기는 기존 pose timeout(기본 0.6초) 후 중립 출력을 선택한다.
  즉시 제동 기능은 아니며 timeout을 0으로 비활성화하면 이 보호 연계가 성립하지 않는다.
- 정상 복구 시 다시 2초 확인한다. 검사 데이터가 없는 상태를 정상으로 간주하지 않는다.
- 고정 yaw offset이나 SIH 정답으로 EKF 위치·자세를 대체하지 않는다.
- `~/status`에 양쪽 yaw, 차이, SIH 수신 나이, 차단 사유를 기록한다.
  다음 `ros2 bag record -a`에도 이 비교 결과가 함께 저장된다.

단위·회귀 테스트 10개 통과. 분리된 ROS 도메인에서 실제 FC 입력으로 브리지
검증: 12초간 정상 odometry 291개, 최종 오차 약 0.54°, EV 송신 0회.
기존 install은 source 심볼릭 링크여서 다음 실행부터 수정 내용이 반영된다.

## 통신 상태

멈춰 있던 MAVProxy를 복구했다. 재부팅으로 ttyACM0→ttyACM1이 되어 USB 고유
경로 `/dev/serial/by-id/usb-CUAV_PX4_CUAV_7_Nano_0-if00`로 연결했다.
로컬 출력 127.0.0.1:14540, 127.0.0.1:14541을 유지한다.
기존 외부 출력 192.168.0.31:14550도 사용자 명시적 승인 후 복구했다.
현재 MAVProxy는 Codex 실행 세션에서 동작한다. 동일 USB로 중복 실행하지 않는다.

## 참고

HIGHRES_IMU의 자기장에는 EKF bias 차감이 포함되므로 SCALED_IMU 원시값과
직접 같은 값으로 취급하지 않는다.
[PX4 HIGHRES_IMU 소스](https://github.com/PX4/PX4-Autopilot/blob/v1.16.0/src/modules/mavlink/streams/HIGHRES_IMU.hpp).

융합 활성·중단 및 heading 일치 조건은
[PX4 1.16.1 mag_control.cpp](https://github.com/PX4/PX4-Autopilot/blob/v1.16.1/src/modules/ekf2/EKF/aid_sources/magnetometer/mag_control.cpp)에 정의돼 있다.
