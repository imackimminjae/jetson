# Lower MPC curvature reference and command contract

This implementation follows the equations in the task specification. Upper MIQP,
interval centering, branch selection, and waypoint decisions are unchanged.
Existing horizon, weights, steering limits, filter tuning, and communication
backends are retained. The fallback gains (0.35 and 1.0) are task-specified gains;
no claim of paper equivalence is made.

## Path transport and spacing

The upper densifier now returns its interpolation query coordinates alongside
its points. `imac_interfaces/msg/PathWithArcLength` contains `nav_msgs/Path path`
and `float64[] s` in one atomic message. The lower controller subscribes to
`<lower_reference_path_topic>/with_arclength` by default
(`lower_use_path_arclength: true`). There is no asynchronous joining of different
path generations. The upper publisher also emits the same Path (same header and
poses) on the original topic for visualization and existing consumers.

For external publishers that only supply `nav_msgs/Path`, set
`lower_use_path_arclength: false`. This explicitly uses cumulative chord lengths
of that input polyline. It does **not** recover the original interpolation
coordinate. The controller never silently switches to this mode when an atomic
path is missing or invalid.

Both modes reject non-finite coordinates, insufficient distinct points, and
non-increasing retained s. Atomic input additionally requires exactly one s per
input pose, before duplicate removal. Consecutive points within 1e-9 m are
removed together with their matching s. An invalid new path invalidates the
current reference; it cannot activate curvature fallback.

Upper/lower configuration both specify Ts=0.10 s. Existing upper spacing is
`max(max(planning_speed,0)*Ts, lower_min_path_spacing_m)`, with minimum 0.05 m.
The current planner uses `target_speed_mps` as planning speed. The checked-in
configuration has upper target 3.0 m/s (0.30 m dense spacing) and lower target
1.5 m/s (0.15 m per step when actually moving at that speed). This is an
existing spacing mismatch; these speed tuning values were not changed.
Thus at planning speed below 0.5 m/s, or when planning speed differs from the
measured lower-controller speed, spatial spacing differs from `v_actual*Ts`.
The last interval can also be shorter. The existing generation rule is retained;
lower MPC uses consecutive dense indices without projection or resampling.
`lower_min_effective_speed_mps` and lower-node `lower_min_path_spacing_m` remain
accepted compatibility parameters but no longer affect the lower computation.
The upper-node spacing parameter continues to control densification.

## Geometry, model, and QP

Headings are forward-segment atan2 values, with the last segment heading repeated
at the endpoint, then unwrapped. Curvature uses centered differences with the
original s (one-sided at each endpoint). This endpoint rule implies zero last
curvature; on a uniformly sampled circle the penultimate curvature is half of
the interior curvature. The horizon repeats the last value beyond the path.

The nearest dense point, with the first point winning a distance tie, defines
the initial lateral and wrapped heading errors. The two-state affine prediction
accumulates `[0, -v*Ts*kappa[k]]` at every step. The actuator option retains its
three-state lag/delay dynamics and uses the same curvature disturbance. It is
an extended model, not the basic two-state model.

DAQP receives `0.5*delta' H delta + f' delta`: the existing factor of two in H
and f is retained. QuadraticProblem forwards H/f without an extra scale. State
costs apply to x(k+1), with absolute-steering and steering-difference costs.
The first difference and its constraint use the previously output, filtered
normalized steering converted to physical radians. This history advances after
QP, fallback, and invalid-input outputs; it is not restricted to QP successes.
It represents commanded steering, not measured actuator feedback.

Zero speed still invokes the same QP. Goal, reset, and supervised stop behavior
remains separate. The fallback first applies the magnitude limit, then the
rate limit about the same previous applied command. QP failure with valid
geometry selects fallback mode 5 before any old-sequence/hold policy. Input
failures retain the existing neutral/hold behavior and never use mode 5.

## Diagnostics

`/debug/control_cmd.linear.x` is selected steering before filtering.
`/control/applied_cmd.linear.x` is output steering in radians; `.linear.y` is
output normalized steering. Existing normalized conversion, virtual command,
duty/PWM, and MAVLink routing are preserved.

`/debug/lower_mpc_trace` retains existing positions and appends new fields.
Indices here are zero-based:

| Index | Meaning |
|---|---|
| 5 | Nearest dense point index (formerly nearest segment) |
| 8 | QP first steering, zero when no QP sequence exists; check index 10 |
| 10 | QP success only; remains 0 for curvature fallback |
| 12 | 0 QP; 1 legacy sequence; 2 invalid-input hold; 3 neutral; 4 stop; 5 curvature fallback |
| 19–20 | Effective steering estimate and previous applied command before this output |
| 24 | Solver-only time; NaN when no solve was attempted |
| 25 | Selected physical steering before the output filter |
| 26 | Current virtual steering output converted to radians |
| 27 | Curvature fallback available (can also be true when QP succeeds) |
| 28 | First reference curvature; NaN without a reference |
| 29 | Applied command used as delta_prev by this solve/fallback |

The MATLAB ROS logger is updated solely as a consumer of this diagnostic
schema: 30 fields, nearest-point column, mode 5, selected/output commands, and
fixed-width rows for older traces. It is not a reference implementation.

## Reproducing validation

From the workspace after sourcing ROS Humble and the workspace overlay:

```sh
CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2 colcon build \
  --packages-select imac_interfaces virtual_control \
  --executor sequential --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select virtual_control --ctest-args --output-on-failure \
  -R 'test_lower_tracking_mpc|test_lower_path_generation|test_lower_steering_actuator_model|test_interval_centering_state|test_miqp_constraint_enforcement'
```

New node tests use isolated ROS domains 174/175 and localhost only. MAVLink is
disabled; no vehicle is connected. Failure injection is compiled only into the
test executable and adds an impossible `0 <= -1` constraint, causing an actual
DAQP failure rather than directly constructing a failed result.

## 실행 결과 (2026-09-23)

최종 빌드: `imac_interfaces`, `virtual_control` 성공. 상·하위 실행 파일,
새 메시지 타입, 테스트 타깃까지 생성되었다. 첫 빌드에서 발견한 vendor의
`inverse` 매크로와 Eigen 헤더 순서 충돌은 Eigen을 먼저 포함하도록 수정했다.
최종 빌드에는 Eigen ARM NEON/MAVLink 외부 헤더의 경고가 남아 있으며 오류는 없다.

| 실제 실행한 CTest 타깃 | 내부 테스트 수 | 결과 |
|---|---:|---|
| test_lower_tracking_mpc | 11 | 모두 통과 |
| test_lower_path_generation | 1 | 통과 |
| test_lower_steering_actuator_model | 3 | 모두 통과 |
| test_interval_centering_state | 13 | 모두 통과 |
| test_miqp_constraint_enforcement | 4 | 모두 통과 |
| 합계 | 32 | 실패 0 |

신규 검증의 구체적 내용:

- 직선 곡률 0, 연속 중복점과 대응 s 제거.
- 반경 4 m 좌·우 원호의 내부 곡률 ±0.25/m, ±pi 방향 통과 시 연속성:
  허용 오차 1e-12 내 통과. 지정한 끝점 규칙의 절반 곡률/0 곡률도 확인.
- 직각 모서리를 가로지르는 보간 및 마지막 0.05 m 구간:
  s=[0, 0.6, 1.2, 1.25]로 차분하고 chord 누적 결과와 다름을 확인.
- 실제 상위 densifyPath()가 위 좌표를 생성하고, 기존 Path와 새 묶음 메시지의
  경로·헤더가 동일하며 s 개수가 맞는 것을 동일 프로세스 ROS pub/sub로 확인.
- 최근접 dense 점에서 초기 오차와 연속 곡률열을 구성하고 끝 값을 반복함을 확인.
- 2상태 모델과 지연 0.2 s 액추에이터 모델 모두 affine+condensed 예측을
  독립 상태식 반복 계산과 비교: 허용 오차 1e-12 내 통과.
- DAQP의 1단계 내부 최적해를 직접 계산한 식과 비교: 허용 오차 1e-7 내 통과.
- v=0, delta_prev=0.30 rad, 변화율 0.05 rad/s, Ts=0.1 s에서 실제 QP 호출,
  모든 조향 크기와 첫 입력을 포함한 단계 변화량 0.005 rad 제한 확인.
- 실제 infeasible QP를 만들고 기존 조향열·hold값을 별도로 채운 상태에서도
  mode=5, QP 성공=0, fallback 사용 가능=1을 확인. 필터 전 선택값,
  필터 후 출력값, 다음 주기의 delta_prev 및 fallback 변화율 제한까지 확인.
- invalid path/pose, stale path/pose, frame mismatch, goal stop, batch stop에서
  곡률 fallback과 구동 throttle을 사용하지 않음을 확인.
- 액추에이터 옵션을 켠 독립 실행 파일의 초기화도 확인. 모든 출력 백엔드를
  포함해 빌드했으며 MAVLink 전송은 테스트에서 비활성화했다.

`git diff --check` 통과. 기존 작업과 비교한 상위 소스 변경은 densification의
s 보존 및 동시 발행 부분뿐이다. 이미 작업 중이던 interval centering 관련
수정/삭제 파일은 되돌리거나 이 작업의 변경으로 포함하지 않았다.

미검증 범위: 실제 차량 및 PX4/SIH 연결 주행, 실장 조향 응답, MAVLink/PWM
하드웨어 전달, MATLAB 로그 소비자의 런타임 실행. 샌드박스의 네트워크 소켓
제한으로 별도 프로세스 간 DDS 통신은 검증하지 않았으며, 새 메시지 테스트는
동일 프로세스 pub/sub 검증이다. 상·하위 목표 속도 차이로 인한 0.30 m 대
0.15 m 간격 불일치는 남아 있으며 튜닝값을 변경하지 않았다.

이번 작업의 변경 파일:

- `src/imac_interfaces/msg/PathWithArcLength.msg`
- `src/imac_interfaces/CMakeLists.txt`, `src/imac_interfaces/package.xml`
- `src/virtual_control/include/virtual_control/lower_path_geometry.hpp`
- `src/virtual_control/include/virtual_control/lower_tracking_mpc_node.hpp`
- `src/virtual_control/include/virtual_control/tracking_control.hpp`
- `src/virtual_control/src/lower_tracking_mpc_node.cpp`
- `src/virtual_control/src/tracking_control.cpp`
- `src/virtual_control/CMakeLists.txt`
- `src/virtual_control/config/tracking_control_split.yaml`
- `src/virtual_control/test/test_lower_tracking_mpc.cpp`
- `src/virtual_control/test/test_lower_path_generation.cpp`
- `matlab/knu_batch/main.m` (진단 메시지 소비 부분만 수정)
- `src/virtual_control/docs/lower_mpc_curvature.md`
