HIL 논문 검증 자료 — 2026-10-09

먼저 열 파일: hil_paper_evidence/HIL_validation_figures.pdf (49페이지)
상세 설명: hil_paper_evidence/REPORT_KO.txt
HIL 구성·실제 설정: hil_paper_evidence/CONFIGURATION_KO.txt
편향 및 미확보 자료: hil_paper_evidence/BIAS_AND_MISSING_KO.txt

전체 분석 자료 다운로드: hil_paper_evidence.tar.gz
원본 8개 배치 다운로드: raw_HIL_logs.tar
전달 파일 SHA256: HIL_DELIVERY_SHA256.txt

최신 시나리오1/2/3 각10회, 총30회와 과거 실패 사례를 정리했다.
목표 접근은 각각9/10,0/10,10/10이며 completed는 목표 정차 성공이 아니다.
시나리오2에서 상위500ms 주기 초과760.8ms 1회가 관측됐다.
편향 HIL 실험, 실제 조향각, 외곽 충돌, 하위 전체 callback 시간은 미확보다.

그림38개PDF·34개PNG,35개 대표 시점, 회차별10페이지PDF를 포함한다.
CSV/JSONL 및 출처·계산법·실행설정·SHA256은 hil_paper_evidence/ 안에 있다.
INDEX.html은 내려받아 로컬 브라우저에서 열 수 있다.
원본 로컬 절대경로는 출처 식별값이며 GitHub 링크는 아니다.
재계산 스크립트의 기존 workspace 의존 경로와 제한은 REPORT_KO.txt 참조.

운영 코드/설정/하드웨어/주행을 변경하지 않고 자료만 추가했다.
