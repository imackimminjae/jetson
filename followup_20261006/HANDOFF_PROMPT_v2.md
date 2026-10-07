ROS 2 + Isaac Sim + PX4 SIH Ackermann 자율주행 프로젝트의 후속 작업을 맡아줘. 아래는 2026-10-06 06시까지의 상태다. 기존 자료와 실제 코드/YAML을 먼저 확인하고 시작해줘. 이전 인계문 followup_20261006/HANDOFF_PROMPT.md도 함께 읽어줘(그 이후 변경만 아래에 정리).

주요 목표
1) 갈림길에서 올바른 분기 선택
2) 주행 중 조향 좌우 떨림 없이 안정 유지

1. 환경과 금지 사항
- 작업 공간 /home/imac/ros2_ws, ROS_DOMAIN_ID=1. 오프라인 시험은 ROS_DOMAIN_ID=177, ROS_LOCALHOST_ONLY=1.
- USB는 기존 MAVProxy 소유(/dev/ttyACM0). 추가 직렬 연결 금지. 주행/ARM/보드 파라미터/펌웨어 업로드 금지.
- yaw가 orientation.z 스칼라로 들어가는 메시지 있음. MAVLink param2 조향 부호는 ROS와 반대.
- mav.tlog는 MAVProxy 재시작 시 덮어써짐(SIH 진실값은 tlog에만 있음). 필요한 구간은 먼저 추출.
- 디스크 여유가 약 3 GB(94~95%)로 작음.
- 다른 세션(Codex 등)이 같은 작업 공간에서 병행 작업한 흔적이 있음(followup_20261006/parameter_tuning, interval_consistency, near_consistency, rollback/20261006_lower_rd150, rollback/20261006_near_interval_application). 손대기 전에 확인할 것.

2. 현재 운영 상태 (YAML 기준, 바이너리는 2026-10-06 01:25 빌드 그대로)
- 상위 분기 모델 묶음 켜짐: upper_cumulative_increment_model, branch_turn_window_at_split, branch_turn_target_absolute (rollback/20261006_upper_increment_model, restore.py)
- upper_planner_rate_hz: 2.0 (사용자가 1.0에서 변경. 5.0은 출발 직후 포화/반복 실패로 나빴음)
- upper_r_dpsi: 2.0 (검증은 3.0 기준. 2.0은 첫 갈림길 S자 조향 포화 증가 경향)
- preview_interval_soft_ratio: 0.8 (05:33 변경, 오프라인 회귀 미검증)
- lower_rd_steering_rate: 150 (다른 세션이 120→150 검증 후 적용. 180은 분기폭 합성 장면 도로 이탈로 회귀 실패)
- wp_switch_eps / wp_switch_max_distance_m: 15 / 20 (원래 10 / 15, 효과 근거 없음)
- lower_steering_actuator_time_constant_sec: 0.74 (rollback/20261003_steering_response_model)
- 사용자가 설정을 수시로 바꿔 가며 주행하므로, 분석 시 bag의 config_snapshot.yaml과 상위 로그(rate, r_dpsi)를 반드시 확인.

3. 주행 결과 요약 (2026-10-06)
- scenario1: 변경 후 대부분 두 갈림길 정분기, 목표 도달. 남은 문제는 첫 갈림길 S자(88~125 m) 조향 포화(15~37%)와 동쪽 갈래 진입(140~149 m) 상위 실패 1회 수준. 최신 05:33 이후 5회 모두 정분기(0551 bag: 목표 0.56 m, 실패 0).
- scenario3: 첫 갈림길 (200,160) 남서 오분기와 마지막 갈림길 (255,140) 북동 미선택이 두 실패 유형.
  1 Hz 12회 전 구간 성공 5/12, 2 Hz 5회 3/4, 5 Hz 0/3. soft 0.8로는 아직 미주행.
  재부팅(04:19) 직후 연속 실패는 원인 미확정(설정/바이너리/지도 정렬/BEV 지연/EKF 헤딩 모두 변화 없음 확인).
- 마지막 갈림길 오분기 원인(실제 BEV 재생으로 확인): 76° 급회전 갈래가 실행 가능한 결정 창이 짧음(1 Hz에서 약 1주기). 갈래가 폭 0 m 조각으로만 보이거나 이미 실행 불가.
- 사용자 가설 "두 번째 분기 진입 전 넓은 도로에서 soft가 걸림" 검증: 사실(기준폭 B가 넓은 도로에서 11→14~20 m로 상승해 14~24 m 단면이 soft). 그러나 soft 폭 상한(15/12/10 m) 변형은 실제 BEV 재생에서 한 사례 개선·한 사례 악화로 일관된 효과 없음(followup_20261006/j2_soft/RESULT.txt).
- 하위 노드 실행기 정지(2.18~2.74 s)가 01:37, 03:52에 재발. 원인 미확정, 계측 필요.

4. 도구
- followup_20261006/scripts/analyze_since.py "START" "END" OUTDIR: 시간 구간의 모든 상위 실행(bag 유무 무관)을 로그·tlog로 매칭해 시나리오, 설정, 분기 결과, 실패 위치, 구간별 조향 지표 출력.
- followup_20261006/j2_soft/dump_j2.py scenarioN TAG...: 갈림길 접근 구간의 단면별 원폭/soft 여부/B/계획 끝 방향 덤프.
- followup_20261006/verify/: 폐루프 하네스(sim_prod), 실제 BEV 재생(replay_prod), 합성/복제/기록 지도 장면 입력.
- 분석 폴더별 RESULT.txt: drive_0137, sc3_0141_0143, sc3_0323_0328, runs_0342_0358, runs_0419_0426, runs_0427_0448, runs_0448_0458, j2_soft.

5. Git 상태
- ros2_ws 로컬 브랜치 hils-cleanup-20260922에 커밋 4a190a4 "rog"(잘된 bag 6개: sc1 034205/035257/050627, sc3 032453/032545/045314). 아직 미푸시.
- 로컬과 원격 hils-cleanup이 갈라져 있음(로컬 d505435, 원격 be33a72→d55f455). 그대로 푸시하면 거부됨 → `git push origin HEAD:logs/good-runs-20261006` 권장.
- 로컬 저장소에 깨진 Codex 참조(refs/codex/turn-diffs/...)가 있어 git fetch 실패. 삭제 여부는 사용자 판단.
- /home/imac/jetson_good_logs: 원격 기준 같은 로그 + README 커밋(6a9318b, 브랜치 logs/good-runs-20261006) 준비됨. 이 장비엔 GitHub 인증이 없어 푸시는 사용자가 직접.

6. 다음 단계 후보
1) scenario3를 현재 설정(soft 0.8, rd 150, 2 Hz)으로 반복 주행해 분석. 이어서 r_dpsi만 3.0으로 되돌린 비교.
2) soft 0.8 오프라인 회귀(합성/복제/강건성/분기폭) 확인.
3) 갈림길·급회전 접근 감속(결정 창 확대, 코너 포화 감소)을 실제 BEV 재생 + 폐루프로 검증 후 적용.
4) 갈래 선택 유지(히스테리시스).
5) 하위 노드 콜백별 실행 시간 계측.

7. 작업 방식
- 확정/강한 가설/미검증 구분. 여러 파라미터를 근거 없이 동시에 바꾸지 않기. 변경 전 백업·해시·덮어쓰기 방지 복원 스크립트.
- 같은 시나리오·같은 갈래 구간끼리 비교. 수동 중단된 주행은 "목표 미도달"과 구분.
