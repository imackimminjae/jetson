# KNU 전체 주행 경로 재구성 (2026-10-09)

S1~S3는 그대로 두고, 기존 S4~S21을 대체할 S4~S12(9개, 2008 m)를 도로 중심선 기반으로 새로 만들었다.
2026-10-09 사용자 요청으로 초기 12개 안의 S7·S8·S14를 빼고 번호를 당겼다. **현재 결과는 `out12/`** (`out12/`은 이전 12개 안, `out12/scenario13~15.csv`는 무효 표시된 잔여 파일 — 설치 금지, 삭제해도 됨). 아직 패키지에 설치하지 않았다(아래 '설치' 참고).

## 근거 데이터
- 도로: `~/.codex/attachments/.../processed_road_map.mat` + `width_samples.csv` (MATLAB `map_gen_knu.m`과 같은 규칙으로 Python 재현: 경계 기반 폭, 접속부 1 m 연장, 27번 도로 연결로).
- 검증: 실제 BEV 누적 지도(S1/S2/S3 배치)와 비교 시 1~2.5 m 이내로 일치 (평행이동 보정 IoU 0.75).
- 참고: 처리된 도로 데이터도 x≈-70(19번 도로 끝)에서 끝난다. 기존 S2 경로 끝 70 m는 지도 밖.

## 생성 규칙
- 45개 도로를 교차점(44개)/T자 접속(25개)에서 잘라 그래프화 (간선 57개, 10 m 미만 자기루프 5개 제외).
- 막다른 끝 또는 긴 도로 중간에서 출발, 120~350 m, U턴 없음, 교차로 회전 ≤95°.
- 경로 선: 중심선 연결 → 곡률 최소화(도로 경계 여유 제약) → 최소 반경 ≥9.5 m, 차량 중심-도로 경계 ≥0.9 m 를 만족하지 못하는 회전은 금지하고 재선정(19개 회전 금지).
- waypoint: 최적화된 선을 단순화(허용오차 0.5 m), 간격 대체로 10~30 m (scenario3 형식), 인접 waypoint 직선도 도로 안.
- 시작: 막다른 끝 6 m 안쪽 또는 첫 교차로 50 m 전. 끝: 막다른 끝 10 m 전 또는 교차로 8 m 전.

## 결과
S1~S3 + S4~S12로 도로 길이 기준 90% 주행 (12개 안 94%). S7·S8·S14 제거로 서쪽 회전교차로 안쪽 연결(10·16·17번 일부)과 15번 북쪽 연결 약 70 m가 추가로 빠짐. 미주행: 25번 동쪽 84 m(실제로는 S1 주행 궤적이 대부분 지남), 북동 회전교차로 서·남쪽 원형 구간 약 40 m(실현 가능한 회전만으로는 진입 불가).
경로별 요약: `out12/routes_summary.json`, 그림: `overview.png`, `routes_final12.png`.

## 설치 시 함께 바꿔야 하는 것
1. `src/virtual_control/data/knu_routes/scenario4~12.csv` ← `out12/scenario*.csv`
2. `knu_start_poses.json`의 scenario4~12 항목 ← `out12/start_entries.json`
3. `src/px4_odom_map_transform.cpp`의 scenario4~12 RouteAnchor ← `out12/px4_anchor_snippet.cpp` (C++ 재빌드 필요)
4. `knu_batch_profile.json` ← `out12/knu_batch_profile.json` (S1~S12)
5. 테스트: `test_knu_global_path.py`의 scenario4 기대값, `test_knu_batch.py`의 MATLAB 대조(scenario5~21) 갱신
6. MATLAB PC: `get_navigation_waypoints.m`(scenario4) / `knu_section_routes.m`(scenario5~12) ← `out12/matlab_cases_snippet.m` (로그용 waypoint, BEV에는 영향 없음)
S13~S21은 기존 파일 유지(프로필에서 제외).
