SIH Ackermann 정지 판정 조사 — 2026-10-02

후속 주행 ULog 분석 완료: 이동 중 내부 정지 오판 확인.
최신 결과는 sess124_analysis/FINDINGS.txt 및 rest_mismatch_evidence.png 참조.
아래 내용은 새 주행 기록을 받기 전의 정지 상태 조사 기록이다.

판정: 주행 중 정지 오판은 아직 미확정. 현재 정지 상태의 판정·GPS 융합은 정상.
ARM, 주행, 파라미터 변경, 펌웨어 업로드, logger on, 로깅 파일 설치를 실행하지 않았다.
기존 MAVProxy가 조사 중 종료된 것을 확인했고, 비어 있는 USB에서 유한한 읽기 전용
조회만 실행한 뒤 연결을 닫았다. 기존 ROS 포트를 바인드하거나 노드를 재시작하지 않았다.

1. 보드 식별과 첨부 파일
보드 simulator_sih status:
  Running Ackermann Rover (no-slip kinematic; input guard v1; local position v2)
보드 ver all: CUAV_7_NANO, PX4 1.16.1,
  94cb2012792b2ae89f0b147cfee53ee31ae550be
SYS_HITL=2, SIH_VEHICLE_TYPE=4, land_detector status=running (rover).
첨부 cuav_7-nano-ackermann-input-fix.px4의 git_hash도 위와 같다.
그러나 첨부 바이너리 문자열은
  Running Ackermann Rover (no-slip kinematic; input guard v1)
이며 'local position v2'는 없다. 따라서 첨부 파일과 실행 중 펌웨어의 동일성은
확인되지 않았고 식별 문자열은 실제로 다르다. 커밋되지 않은 소스 변경은 Git
해시만으로 복원할 수 없다. attached_firmware_metadata.json에 SHA256을 저장했다.

2. 직접 조회한 내부 상태 (04:56:38.900~04:56:56.377 KST)
순차 listener 조회이며 동시 샘플이라고 간주하지 않는다. 같은 조회 시간 동안
수신한 HIL 실제 속도 530개는 모두 0m/s였다. uORB 자체 timestamp도 보존했다.
- SIH groundtruth: NED x=-333.23746, y=557.22717m; vx=vy=vz=0.
- vehicle_land_detected: landed=true, ground_contact=true, at_rest=true.
- estimator_status_flags: cs_vehicle_at_rest=true, cs_in_air=false,
  cs_gnss_pos=true, cs_gnss_vel=true.
- estimator_gps_status: checks_passed=true, 모든 check_fail_* = false.
  수평 위치 드리프트 0.01404m/s, 필터된 수평 속도 0.01935m/s.
- vehicle_local_position: xy_valid=true, v_xy_valid=true,
  vx=-0.00660, vy=0.01396m/s; xy_reset_counter=58, vxy_reset_counter=58.
  이 카운터만으로 어떤 주행에서 언제 reset됐는지는 알 수 없다.
- actuator_armed: armed=false, prearmed=true (lockdown=true).
- actuator_outputs_sim: 16개 출력 모두 0, timestamp=8272476351us,
  조회 시 마지막 갱신 약195.4초 전. 정지 중 오래된 0 입력을 이동 명령으로 해석하지 않는다.
현재 정지 중 남아 있는 GPS 검사 실패 항목은 없다. 이전 실패의 정확한 복구 시각은
이 단발 조회로 역산할 수 없다.

3. 기존 기록의 한계
- 오늘 로스백 20개의 topics 테이블에 요청한 uORB 내부 상태 토픽이 없다.
  bag_topic_audit.json 참조. ROS의 /px4/sih/odom을 SIH groundtruth로 간주하지 않았다.
- 기존 MAVLink 로그의 HEARTBEAT/EXTENDED_SYS_STATE/ESTIMATOR_STATUS는
  vehicle_land_detected.at_rest / cs_vehicle_at_rest를 직접 제공하지 않는다.
- SD카드 LOG_ENTRY 1095/1095개 수신. 기록상 최신 항목은 ID1094,
  UTC time 기준 2026-09-13 04:11:00 KST, 1791753bytes.
  /fs/microsd/log/2026-09-12/19_10_26.ulg.
  오늘의 정지→이동→정지 구간에 해당하는 ULog를 찾지 못했다.
- logger는 mode=all, subscriptions=112, Not logging.
  SDLOG_PROFILE=1, SDLOG_MODE=0. 현재 로깅은 ARM에 연동된 기본 모드이다.
- /fs/microsd/etc/logging은 현재 존재하지 않는다.
- 기본 로그 토픽 목록은 estimator_gps_status, actuator_outputs_sim을 명시적으로
  포함하지 않는다. 단순 logger on만으로 모든 요청 항목이 모인다고 가정하지 않는다.

4. 준비된 기록 방법 — 아직 보드에서 실행하지 않음
logger_topics.txt에는 필요한 7개 핵심 토픽과 GT 자세, GPS 원본, IMU 상태,
EKF 선택 인스턴스 등 보조 토픽을 포함했다. interval 단위는 ms이며 0은
발행되는 모든 업데이트를 기록한다. EKF 내부 토픽은 instance 0~3을 준비했다.
이 파일은 기본 토픽 프로파일에 덧붙이는 것이 아니라 대체한다.

사용자가 다음 기록을 시작하기로 했을 때만 진행한다.
A. 기존 파일이 생겼다면 먼저 백업하고, 준비 파일을 SD카드의 다음 위치로 복사:
   /fs/microsd/etc/logging/logger_topics.txt
   MAVProxy가 연결된 상태에서 그 콘솔을 사용하는 경우:
     module load ftp
     ftp mkdir /fs/microsd/etc
     ftp mkdir /fs/microsd/etc/logging
     ftp put /home/imac/ros2_ws/px4_rest_audit_20261002/logger_topics.txt /fs/microsd/etc/logging/logger_topics.txt
   이미 존재하는 디렉터리의 mkdir 실패와 파일 전송 실패는 구분한다.
   전송 완료를 확인한 뒤 아래 명령을 실행한다. 펌웨어 파일 업로드가 아니다.
B. PX4 NSH 콘솔에서 로거만 재시작하고 명시적으로 기록을 켠다:
     logger stop
     logger start -m file
     logger on
     logger status
   여기서 on은 ARM을 수행하지 않고 '기록 시작 조건'만 override한다.
   logger_topics.txt 읽기 성공과 실제 .ulg 파일 기록 상태를 확인한다.
   분석기가 필수 토픽 누락을 다시 검사하므로 기록만 했다고 완전한 자료로 간주하지 않는다.
C. 기록을 유지한 상태에서 사용자가 직접 정지→이동→정지를 수행한다.
   권장 관측 창: 시작 정지 약30초, 평소 방식의 주행(직선 구간 포함), 종료 정지 약60초.
   시간을 채웠다는 사실로 정상 복구를 보장하지 않는다. 실패 플래그가 내려가는지 관측한다.
   이 준비물에는 ARM·주행·액추에이터 명령이 없다.
D. PX4 NSH 콘솔:
     logger off
     logger status
   상태에 표시된 파일을 QGC 또는 MAVProxy FTP로 PC에 복사한다.
   MAVProxy 예시에서 실제 경로와 저장 파일명을 사용:
     ftp get /fs/microsd/log/실제폴더/실제파일.ulg /home/imac/ros2_ws/px4_rest_audit_20261002/drive.ulg
E. 임시 topic 파일은 기록 종료 후 제거하거나 이전 백업을 복구하고 로거를 원래
   시작 명령으로 재시작해야 기본 프로파일로 돌아간다. 이 변경은 이번 조사에서
   실행하지 않았으며, SDLOG_PROFILE/SDLOG_MODE 등의 파라미터도 바꾸지 않는다.

5. 분석 실행 — PC에서만 실행
/home/imac/ros2_ws/px4_rest_audit_20261002/.venv/bin/python \
  /home/imac/ros2_ws/px4_rest_audit_20261002/analyze_ulog.py \
  /home/imac/ros2_ws/px4_rest_audit_20261002/drive.ulg \
  --output /home/imac/ros2_ws/px4_rest_audit_20261002/drive_analysis

결과: aligned.csv, summary.json, timeline.png.
- 모든 데이터는 보드 부팅 이후 timestamp(us)로 정렬한다.
- timestamp_sample도 보존하고, Boolean을 보간하거나 미래 값을 과거로 채우지 않는다.
- 실제 속도>0.2m/s, 두 내부 at_rest=true, 신선한 샘플이 0.5초 이상 겹칠 때만
  '움직임/정지 판정 불일치 관측'으로 분류한다. 이 기준은 분석 기준이지 PX4 변경값이 아니다.
- 불일치가 없는 경우에도 로그 범위·샘플 신선도·dropout을 확인해야 하며 정상으로 단정하지 않는다.
- at_rest, cs_vehicle_at_rest, 각 GPS check_fail_*, GNSS 융합, xy_valid/v_xy_valid,
  reset counter의 변화 시각을 events에 보존한다.
- 각 정지 구간의 GPS 실패 항목, 마지막 2초에 남아 있는 실패, 개별 실패 해제와
  수평 유효성이 처음 관측된 시각을 기록한다.
- actuator 출력과 마지막 갱신 시각/age를 함께 기록한다.
- EKF 인스턴스가 바뀌면 경고한다. --estimator-instance로 각각 분석하고
  selector 기록과 대조한 뒤 vehicle_local_position의 원인을 판단한다.
- 합성 ULog로 읽기→정렬→CSV/그래프 생성까지 검증했다. 합성 자료는 실제 주행 증거가 아니다.

6. 현재 커밋 소스에서 확인한 조건부 원인 경로
reference_source/에는 보드가 보고한 해시의 공개 소스를 보관하고 URL/SHA256을 기록했다.
실제 수정 작업 트리와 일치한다고 보장하지 않는다.
- RoverLandDetector.cpp: 일반 분기에서 landed = !_armed; ground_contact는 true.
- LandDetector.cpp: 회전/진동 기반 _at_rest와 landDetected를 AND해서 at_rest 발행.
  기본 경로는 실제 병진 속도를 직접 사용하지 않는다.
- EKF2.cpp UpdateSystemFlagsSample: flags.at_rest=vehicle_land_detected.at_rest,
  flags.in_air=!vehicle_land_detected.landed.
- gps_checks.cpp: 지상+vehicle_at_rest일 때 위치·속도 drift 검사.
  지상+움직임 분기에서는 필터를 reset하지만 이전 drift 실패 플래그는 유지한다.

이는 불일치가 일어날 수 있는 경로 설명이며, 이번 주행에서 실제 발생했다는 증명은 아니다.
주행 중 내부 플래그 불일치가 확인되기 전에는 펌웨어 수정안을 확정하거나 적용하지 않는다.
확인될 경우의 최소 수정 방향은 SIH Ackermann(SYS_HITL=2, SIH_VEHICLE_TYPE=4)에서만
신선한 groundtruth 속도로 기존 '정지' 판정을 거부하는 보조 조건을 넣는 것이다.
실제 정지 확인에는 속도 히스테리시스/유지 시간을 두고, groundtruth가 오래되거나
유효하지 않으면 정지라고 단정하지 않도록 설계한다. 실제 차량/다른 SIH 모델,
landed/ground_contact/armed 의미, GPS 검사 마스크·기준은 유지한다.
정확한 함수 위치와 stale 처리 정책은 현재 수정 소스와 다음 ULog 확인 뒤 결정한다.
