# 검증 상태 — 구현 보관 단계

2026-10-19 사용자 요청에 따라 **추가 테스트 실행을 중단**했다.

작업 도중 초기 코어와 ROS 연결 일부를 확인했지만, 별도 실행 파일 분리 및 마지막 종점
접근 처리를 포함한 최종 코드 전체의 검증 결과로 간주하지 않는다. 이 상태는 구현 작성본이다.

* 기존 `tracking_control.cpp`, `tracking_control.hpp`는 이번 작업에서 수정하지 않았다.
* 기존 `tracking_control_split.launch.py`에 잠시 추가했던 선택 기능은 원복했다.
* 새 노드는 `nvp_planner_node`, `mpcc_planner_node`이며, 새 선택 진입점은
  `planner_comparison.launch.py`이다.
* 최종 코드의 추가 단위/통합 테스트, Isaac/PX4 주행 평가 및 비교 성능 평가는 미실행이다.
* 테스트 소스와 재현 명령은 보관했으며 이후 명시적으로 검증을 진행할 때 사용할 수 있다.

구현, 빌드 의존성, 실행 방법, 논문 대비 차이와 한계는 `planner_comparison.md`를 참고한다.
