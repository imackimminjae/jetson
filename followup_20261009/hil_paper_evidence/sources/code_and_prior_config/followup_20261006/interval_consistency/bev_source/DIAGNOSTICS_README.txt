첨부 BEV 생성기의 진단 전용 사본 — 2026-10-06

파일
- rgb_to_occupancy.original.cpp: 첨부 원본 바이트 보존.
- rgb_to_occupancy.diagnostics.cpp: 진단 기능 추가 사본.
- diagnostics.patch: 원본과의 차이.
- diagnostics_sha256.json: 원본/사본/패치 해시.

현재 적용 상태
이 PC에는 해당 ROS 패키지/실행 중 생성기가 없어 설치하거나 재시작하지 않았다.
현재 제어기 소스/파라미터도 변경하지 않았다.
로컬 record_drive_debug.sh의 기록 대상에 /debug/bev_source_timing만 추가했다.
새 토픽이 없어도 기존 토픽 기록을 시작할 수 있으며 새 생성기가 있어야 진단 데이터가 쌓인다.

진단 사본이 하는 일
새 파라미터 publish_bev_source_diagnostics=true가 기본값이며 false로 끌 수 있다.
기존 분할/축소/회전/TF 선택/OccupancyGrid stamp 설정과 픽셀 결정식을 유지한다.
정상 처리된 이미지마다 /debug/bev_source_timing (Float64MultiArray, best effort, depth 10)를 보낸다.
실패한 프레임/건너뛴 프레임에는 레코드가 없으며 image_seq 증가량으로 공백을 볼 수 있다.
영상이나 지도 픽셀 자체를 추가 발행하지 않는다.

필드(0부터, 메시지 layout.dim[0].label에도 이름 저장)
 0 image_seq: 수신한 전체 이미지 개수(처리 생략 프레임 포함)
 1 image_stamp_sec: RGB 원본 시각
 2 callback_ros_sec: 이미지 콜백 진입 ROS 시각
 3 after_grid_publish_ros_sec: 격자 publish 호출 이후 ROS 시각(네트워크 수신 시각은 아님)
 4 work_ms_to_grid: 콜백 시작부터 격자 발행 뒤까지 steady clock 경과시간
 5 tf_stamp_sec: 실제 조회한 camera<-Vehicle TF의 시각(정적 TF라면 0일 수 있음)
 6 components: 최대 영역 선택 전 연결 영역 개수
 7 total_area_px: 선택 전 모든 영역 면적 합
 8 selected_area_px: 남긴 가장 큰 영역의 면적
 9 second_area_px: 두 번째 영역 면적
10 selected_label: 해당 프레임 안의 연결 영역 번호
11 keep_largest
12 road_gray_max
13 white_line_gray_min
14 morph_ksize: 설정값(원본 처리에서는 최소1, 짝수면+1 보정)
15 zero_grid_stamp
16 use_latest_tf
17 process_every_n_frames
18 use_sim_time
19 grid_enabled
20..25 ROI x_min,x_max,y_min,y_max,BEV resolution,ground_z
26..30 occupancy width,height,rotate_ccw90,flip_lr,invalid_as_unknown
keep_largest=false이면 연결 영역 통계 6..10은 -1(원본이 영역 계산을 하지 않음).

해석 주의
- image_stamp와 callback_ros가 같은 시간 기준임을 확인한 뒤 차이를 지연으로 해석할 것.
  Isaac 시뮬레이션 시간과 호스트 wall time을 바로 빼면 안 된다.
- work_ms_to_grid에는 이후 선택적 resized 이미지/미리보기 처리는 포함되지 않는다.
- selected_label은 프레임 간 동일 갈래 ID가 아니다. 번호 변경만으로 영역 뒤집힘을 판정하지 않는다.
- 상위 두 면적이 비슷한 프레임에서 실제 격자와 후보 선택이 어떻게 바뀌는지 같이 비교한다.
- camera<-Vehicle이 고정 외부변환이면 latest TF와 영상시각 TF는 같은 값일 수 있다.
  use_latest_tf=true만으로 지연 원인이라고 판정하지 않는다.

다른 PC 적용 전
실제 소스가 첨부 원본과 다르면 패치를 검토해서 해당 파일에 반영할 것.
기존 실행 YAML을 유지한다. keep_largest_component, 색상 문턱, 회전, ground_z를 임의 변경하지 않는다.
std_msgs 의존성이 해당 패키지에 없다면 package.xml과 CMakeLists.txt의 기존 실행 대상에 추가해야 한다.
이 PC에서는 전체 C++ 소스의 구문 검사만 통과했다. 다른 PC의 실제 패키지 빌드/실행은 미검증이다.
원본/현재 파일 백업과 해시를 먼저 남기고, 이후 수정 파일이 달라졌을 때 덮어쓰지 않는 방식으로 복원할 것.

다음 사용자 주행 기록
출발 전부터 기존 bag 기록을 시작하고 이번 진단 토픽이 실제 포함되는지 확인한다.
가능하면 짧은 문제 구간의 RGB/CameraInfo/TF도 별도로 확보한다(현재 경량 기록기는 RGB를 기록하지 않음).
동일 시나리오/갈래로 비교하고, tlog는 MAVProxy 재시작 전에 별도 보존한다.
이 진단 사본은 주행/ARM/직렬 연결/펌웨어/보드 파라미터를 제어하지 않는다.
