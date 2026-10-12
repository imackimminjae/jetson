# 엣지 케이스 그림 모음 (2026-10-09, 주행 중 저부하로 작성)

이미 추출된 JSON/pickle만 사용(bag 미접근), 코어 0·nice 19.

| 번호 | 그림 | 내용 |
|---|---|---|
| 1·2 | case01_02_gore_reference.png | 참조점이 갈래 사이 섬 위 → B 후보 없음, 섬 끝 0.1 m 조각 구간 |
| 3 | case03_viewpoint_narrow.png | 같은 지점이 차량 위치에 따라 2.0 m / 3.7 m로 읽힘 |
| 4 | case04_s3_corner_truncation.png | 시나리오3 코너 안쪽 접근 시 갈래 단면 3.5~4.6 m로 잘림 → QP 불가 |
| 6 | case06_B_lock.png | B가 2 m로 고정되어 회복 불가 (1.5×B 거부 규칙) |
| 7 | case07_confirm_side_effect.png | B 감소 지연(confirm) → 북쪽 오분기 증가 (폐루프 시뮬) |
| 10 | case10_phase_sensitivity.png | 시작점×상위 계획 위상별 성패 (폐루프 시뮬) |
| 11·12 | case11_12_s2_map_and_island.png | 시나리오2 경로 끝이 지도 밖, 경로 선이 섬을 관통 |
| 15 | case15_acute_junctions.png | 예각 교차로 104°/116°/98°: 조향 한계로 도로 안 회전 불가 |
| 16 | case16_roundabout_radius.png | 서쪽 회전교차로 원형 도로 반경 vs 최소 회전 반경 9.1 m |
| 17 | case17_dense_waypoints.png | 5 m 간격 waypoint가 끝점 기준점보다 뒤에 놓임 (개념도) |
| 18 | case18_recorder_gap.png | 녹화 파일 전환 2.32 s 공백 → 배치 중단 |
| 20 | case20_coasting.png | completed 후 제동 없이 32~37 m 굴러감 |

그리지 않은 것:
- 5 (k7에서 갈래 빠짐): 사용자 제공 그림 및 followup_20261006/verify/world_j2_zoom.png
- 8·9 (분기 관성, QP 모델 불일치): 구조 설명이라 기존 자료 참고 (followup_20261006/verify, HANDOFF_20261006_branch_steering.txt)
- 13 (BEV 원점 2.95 vs 0): 원자료 bag 필요 → 주행 후
- 14 (재부팅 후 EKF heading): followup_20261006/figures/ekf_sih_heading_after_reboot_20261005.png
- 19 (무거운 I/O 시 노드 정지), 21 (setup/pose timeout): 원인·자료 미정리
