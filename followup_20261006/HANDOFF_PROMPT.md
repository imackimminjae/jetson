ROS 2 + Isaac Sim + PX4 SIH Ackermann 자율주행 프로젝트의 후속 작업을 맡아줘. 아래는 2026-10-06까지의 변경점과 근거다. 기존 자료와 실제 코드를 먼저 확인하고 시작해줘.

주요 목표
1) 갈림길에서 올바른 분기 선택
2) 주행 중 조향 좌우 떨림 없이 안정적으로 유지

1. 환경과 금지 사항
- 작업 공간: /home/imac/ros2_ws, ROS_DOMAIN_ID=1. 오프라인 시험은 ROS_DOMAIN_ID=177, ROS_LOCALHOST_ONLY=1.
- USB는 기존 MAVProxy가 소유한다(/dev/ttyACM0, --out 14540/14541). 추가 직렬 연결 금지.
- 임의 주행, ARM, 강제 ARM, 보드 파라미터 초기화, GPS 검사 비활성화, 펌웨어 업로드 금지.
- 일부 PoseStamped/Odometry는 orientation.z에 yaw 라디안을 직접 넣는다. 일반 쿼터니언으로 해석하지 마.
- MAVLink COMMAND_LONG 187의 param2(조향) 부호는 ROS 조향 부호와 반대다(NED yaw rate 부호와는 같다).
- mav.tlog는 MAVProxy 재시작 시 덮어써진다. 분석할 구간은 먼저 복사해 둘 것.

2. 적용된 변경 (시간순)

2-1. 2026-10-03: 하위 MPC 조향 응답 모델 시정수 0.05 → 0.74 s
- 파일: src/virtual_control/config/tracking_control_split.yaml, lower_steering_actuator_time_constant_sec: 0.74
- 근거: 현재 펌웨어의 실제 조향 응답 시정수가 0.70~0.9 s(여러 주행에서 재식별). 펌웨어 read_motors 입력 필터가 새 출력 이벤트 때만 SIH dt로 갱신되는 것이 강한 원인 가설(최종 펌웨어 소스는 다른 PC에 있어 미대조).
- 실주행 확인: 하위 내부 조향 추정 vs SIH 실제 RMSE 8.1 deg → 0.22 deg, 같은 갈래 조향 포화 32% → 2~4%.
- 복원: python3 /home/imac/ros2_ws/rollback/20261003_steering_response_model/restore.py
- 펌웨어 필터를 고치면 0.74를 그대로 두지 말고 재식별할 것.

2-2. 2026-10-06 01:25: 상위 플래너 분기 모델 묶음 (적용, 빌드 완료)
- 파일: src/virtual_control/src/tracking_control.cpp, include/virtual_control/tracking_control.hpp, config/tracking_control_split.yaml(upper_planner_node)
- 코드 기본값은 모두 기존 동작(false). YAML에서 켬:
    upper_cumulative_increment_model: true
    branch_turn_window_at_split: true
    branch_turn_target_absolute: true
    branch_split_continuity_m: 6.75
    upper_r_dpsi: 3.0   (기존 1.5)
- 내용:
  a) 누적 증분 모델: 상위 QP 위치 예측(G)이 입력을 한 구간에만 반영하던 것을, 속도/방향 증분이 이후 모든 구간에 누적되도록 변경. 기존에는 위치 예측은 "구간 방향 편차", 조향 제약과 회전 비용은 "스텝당 증분"으로 같은 변수를 다르게 해석했다. 그 결과 계획은 참조 대비 약 28.5 deg 이상 벌어질 수 없었고(기록 최대 27.2 deg), 실제 계획 꺾임은 조향 한계를 넘기도 했다(56개 중 7개, 최대 35 deg).
  b) 회전 비용 창: 첫 다중 후보 스텝 대신 "한 갈래가 둘 이상으로 이어지는 스텝"에 배치(인접 단면 구간 간 최소 거리 <= 6.75 m를 같은 갈래로 판정). 이전 갈림길 잔상 후보에 창이 걸리던 문제 해결.
  c) 절대 회전 목표: 목표를 "진입 도로 방향 + 비율 x 도로 회전각"(차량 기준)으로. 참조가 이미 꺾였을 때 이중 요구와, 웨이포인트 쌍 전환(10 m 전) 직후 목표 상실 해결.
  d) upper_r_dpsi 3.0: 누적 모델에서 참조 대비 곡률 차이 비용. 1.5로는 분기폭 합성 장면에서 지선 오분기.
- 네 요소는 묶음으로만 검증됨. 하나만 켜면 회귀 확인됨(누적 모델 단독: 분기폭 장면 지선 이탈).
- 시작 로그로 적용 확인: "upper branch model: cumulative_increment=1 turn_window_at_split=1 turn_target_absolute=1 split_continuity=6.75m r_dpsi=3.000"
- 복원:
    빠른 복원(YAML만, 재빌드 불필요, 이전 플래너와 궤적 동일 검증됨): python3 /home/imac/ros2_ws/rollback/20261006_upper_increment_model/restore.py
    전체 복원(소스/헤더/YAML + 재빌드): restore.py --full
    이후 파일이 바뀌었으면 덮어쓰지 않고 중단한다. 백업/해시/빌드 로그는 같은 폴더.
- 빌드 명령: colcon build --symlink-install --packages-select virtual_control --parallel-workers 1 --cmake-args -DBUILD_TESTING=ON (설치 YAML은 소스 심볼릭 링크)

3. 검증 근거
- 오프라인(실제 상위/하위 C++, 하드웨어 출력 없음, 차량 tau 0.74): followup_20261006/verify
  scenario1 두 번째 갈림길 복제 지도 동쪽 정분기 0/3 → 3/3, 북쪽 대조 3/3 유지
  직선 Y자 12조건 12/12, 기존 곡선/분기폭 회귀 장면 통과
  강건성(차량 tau 0.5/1.0, 헤딩 잡음 2 deg) 원본 8~9/12 → 12/12, 실패 5 → 0
  운영 소스로 재컴파일한 결과가 검증 사본과 172회 궤적 동일, 스위치 off = 이전 플래너와 궤적 동일
  기존 C++ 테스트 6종 통과
- 실주행 (모두 변경 적용 확인):
  scenario1 2026-10-06 01:37 (drive_debug_20261006_013717_light): 두 번째 갈림길 첫 정분기(이전 3회 모두 북쪽 오분기), 최종 웨이포인트 1.5 m 이내 도착. R3(10-05 23:45) 대비 횡오차 p95 구간별 0.33→0.29, 0.84→0.74, 1.45→0.43, 0.95→0.73 m. 조향 한계 초과 계획 0개. 첫 갈림길 S자/동쪽 우회전에서 포화 각 1회(반복 진동 아님). 상위 실패 1회(동쪽 갈래 진입 중).
  scenario3 01:41 / 01:43 (drive_debug_20261006_014143_light, _014303_light): 경로대로 두 갈림길 정분기, 목표 도달. 끝부분 전 상위 실패 1회/0회. 같은 BEV를 이전 플래너로 재생하면 끝부분 전 각 10회 실패.

4. 확인된 주요 사실 (원인 분석)
- scenario1 두 번째 갈림길 오분기 원인: 회전 비용 창이 첫 갈림길 왼쪽 갈래 잔상(k=1)에 걸림 + 위치 예측/제약 불일치로 참조 대비 50 deg 갈래 도달 불가. 회전 비용 계수만 올리면 오히려 정답 갈래 비용 증가.
- 제어기 속도 편향(+0.25~0.45 m/s): PX4 LOCAL_POSITION_NED time_boot_ms가 30/40 ms로 교대 → 브리지가 수신 시각으로 차분해 0.1 s 톱니파 → 하위 10 Hz가 고정 위상으로 표본. 01:37 주행에서 +0.07~0.10은 위상 우연. 미수정.
- 보드 재부팅 직후 첫 주행: EKF 헤딩이 SIH 대비 7~12 deg 틀어진 채 출발 후 주행 중 수렴(헤딩 검사 문턱 15 deg라 통과). 이런 주행은 비교에서 제외.
- 1 Hz 상위 실패 1회 → 경로 나이 2 s > 하위 경로 timeout 1.5 s → 0.5 s 중립 구조는 그대로.

5. 미해결 과제 (우선순위 제안)
1) 근거리 계획 일관성(조향 안정): 넓은 갈림길 입구에서 k=1~2 단면 후보가 둘로 갈리고 선택이 주기마다 0/1로 바뀜 → 근거리 계획 좌우 이동 → 조향 흔들림(scenario3 01:41 포화 14.7%, yaw +/-12 deg). 근거리 계획을 직전 계획에 묶는 비용 또는 근거리 후보 선택 히스테리시스를 오프라인 검증부터.
2) 하위 노드 실행기 정지: 01:37 주행 01:37:50.35~53.09 2.74 s 동안 제어/keepalive 모두 정지(단일 스레드 실행기). 펌웨어 0.5 s 보호로 입력 0, 속도 4.85→3.71 m/s. 같은 유형 10-03 05:59 0.52 s, 05:46 ULog 0.86 s. 직전 계산 시간 정상, 시스템 로그 없음, 원인 미확정(스왑 838 MB 사용 중, 미검증). 먼저 콜백별 실행 시간/타이머 지연 계측.
3) 급회전/분기 감속: 6 m/s 고정이라 우회전 yaw rate 30 deg/s(한계 약 38). 곡률/분기 기반 목표 속도 하향 검토.
4) 좁은 갈래 soft 축소: 원폭 4~5.6 m가 soft 0.7로 1.7~2.2 m. 잘린 단면 soft 처리와 B 급변 문제도 남아 있음.
5) 속도 경로: PX4 속도 직접 전달 또는 메시지 시각 차분.
6) 펌웨어 Ackermann 입력 필터를 매 SIH 루프 실제 dt로(0.5 s 입력 보호 유지) → 이후 0.74 재식별.
7) 목표 판정 결함: remaining=0 OR 조건으로 멀리서 도착 처리 가능. 목표 정지 후 MAVLink 중립만 보내 관성 진행(direct 10~20 m까지 증가).
8) 지도 header.stamp 0, 카메라 투영/기준점 정합 미검증.

6. 자료 위치
- 분석/스크립트: /home/imac/ros2_ws/followup_20261005, /home/imac/ros2_ws/followup_20261006
  verify/: 변형 사본, 폐루프 하네스(sim_prod), 실제 BEV 재생(replay_prod), 시험 장면, VERIFY_RESULT.txt
  drive_0137/RESULT.txt, sc3_0141_0143/RESULT.txt, figures/
- 10-03 전수 조사: /home/imac/ros2_ws/full_audit_20261003/RESULT.txt
- 롤백: /home/imac/ros2_ws/rollback/20261003_steering_response_model, /home/imac/ros2_ws/rollback/20261006_upper_increment_model
- 참고: rollback/20261006_upper_increment_model/RESULT.txt는 권한 문제로 작성되지 않았다. 내용은 이 문서 2-2, 3절과 같다.

7. 작업 방식
- 확정 사실, 강한 가설, 미검증 사항을 구분해서 보고할 것.
- 여러 파라미터를 근거 없이 한 번에 바꾸지 말 것. 묶음 변경이 필요하면 왜 분리할 수 없는지 근거를 남길 것.
- 변경 전 백업과 해시, 덮어쓰기 방지 복원 스크립트를 준비할 것.
- 다음 주행 분석은 bag(출발 전부터 기록)과 tlog를 함께 쓰고, 시나리오/갈래가 같은 구간끼리 비교할 것.
