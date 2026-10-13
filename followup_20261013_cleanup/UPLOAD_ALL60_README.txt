2026-10-09 全12シナリオ/전체12개 시나리오 각5회 HIL 추가 자료

브랜치 logs/archive-20261006의 기존 2026-10-08 자료는 유지.
이번 batch: 20261009_070714_all_d8c3cf, 60회 실제출발, 접근25/off_route33/stopped2.

우선 읽기: batch_070714_review/RESULT.txt 및 INDEX.html
전 회차 그림: figures/scenario_all_attempts.pdf (12페이지, 시나리오별 개별5회 포함).
전체 그림: figures/all_60_routes.png
집계/시행표: tables/per_scenario.csv, tables/per_attempt.csv
추종/시간: figures/state_error_command_histories.pdf, runtime.png, tables/runtime.csv
신호: signals/*.csv.gz, phase/run 연결: phases.json
실행 설정: configs/effective_config.yaml 및 routes/ 사본.

HIL_all60_20261009_review.tar.gz: 위 분석 자료 묶음.
raw_HIL_all60_20261009.tar.gz: 최신60회 + 별도 startup 실패2세션 원본, 총3개 batch.
HIL_ALL60_SHA256.txt: 두 압축파일 체크섬.
해제: tar -xzf raw_HIL_all60_20261009.tar.gz
원본 구조: drive_batches/.../bag/, effective_config.yaml, manifest.json, results/.../summary.csv.

completed는 목표 영역 접근이며 목표 정차 성공이 아님. off_route는 waypoint 기준.
시작실패 두 세션은60회 집계 제외. 현재 기본 YAML 해시가 주행 당시와 달라 실제 전달 설정 사용.
이번 자료는 빠른 결과 정리이며 원인/차량 외곽 충돌/편향실험 입증 자료가 아님.
