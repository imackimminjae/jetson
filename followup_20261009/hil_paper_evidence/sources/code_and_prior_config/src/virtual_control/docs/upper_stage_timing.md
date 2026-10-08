# 상위 계획기 지연 측정

상위 계획기를 새 빌드로 재시작하고 평소처럼 주행하며 아래 명령으로 녹화한다.

```bash
bash /home/imac/ros2_ws/src/virtual_control/scripts/record_drive_debug.sh
```

문제가 나타난 뒤 Ctrl+C로 종료한다. 이 녹화는 이미지와 포인트 클라우드를 제외하고 `/debug/upper_stage_timing` 및 기존 `/debug/upper_solver_timing`을 포함한다. domain은 1이다. 실시간 확인:

```bash
ROS_DOMAIN_ID=1 ros2 topic echo /debug/upper_stage_timing --qos-reliability best_effort
```

`upper_stage_timing`은 String 안의 JSON이다. `cycle_seq`는 모든 타이머 호출, `plan_seq`는 기존 solver timing/branch event의 계획 시도 번호다. 입력 부족으로 계획 전에 반환하면 `plan_seq=0`이다. `t_start`는 ROS 시각이며, 소요 시간 측정에는 단조 증가 시계를 사용한다.

- `wall_ms`: 해당 단계의 실제 경과 시간.
- `cpu_ms`: 같은 스레드가 CPU를 사용한 시간. wall만 크면 대기 또는 스케줄링 지연의 단서이며, 특정 원인을 확정하지는 않는다.
- `start_interval_ms`: 이전 호출 시작부터 이번 시작까지의 간격. cycle 자체는 짧은데 간격이 길면 타이머 밖의 콜백이나 실행 대기도 조사한다.
- `previous_report_ms`: 직전 계측 JSON 작성 및 발행 비용. 현재 계측 발행 비용은 다음 샘플에서 확인한다.
- `outcome`: 성공, 목표 도달, 또는 조기 반환. 실패의 구체적인 이유는 같은 `plan_seq`의 branch event/기존 로그로 확인한다.

`stages`는 이전 측정 지점부터 이름에 해당하는 작업 완료까지의 시간이다. 합계는 전체 `wall_ms`/`cpu_ms`와 일치한다. 초기 snapshot·입력 확인, preview 생성, interval 추출, solver 전체 호출, 진단 발행, 경로 변환·저장·발행, 로그 출력, 지역 변수 해제까지 구분한다. 조기 반환 시 실행한 단계만 기록한다.

특히 `solve_upper_full_call`은 solver 함수의 후처리와 지역 변수 해제까지 포함한다. 기존 solver 내부 시간과 비교하여 내부 타이머 밖의 지연도 찾을 수 있다. `solver_metric_publish`, `upper_trace_publish`, `sparse_path_publish`, `dense_path_publish`, `branch_event_publish`, `legacy_timing_publish`는 각각 발행 구간이다. `road_turn_log`, `result_log`는 로그 호출 구간이다.

계측 자체는 best-effort로 발행하므로 수신 측에서는 유실될 수 있다. 한 주기가 반환해야 보고서가 나오므로 영구 정지/강제 종료 시 마지막 실행 단계는 이 토픽만으로 확인할 수 없다. 기존 2026-09-23 22:47:29의 1.52초 지연에는 이 단계별 정보가 없으므로, 이번 계측을 켠 상태에서 재현해야 위치를 좁힐 수 있다.
