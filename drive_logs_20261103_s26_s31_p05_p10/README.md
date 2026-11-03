# +5°·+10° 원본 로그

기록 당시 세션 폴더를 파일 내용 수정 없이 보관했다. 출발 전 중단 폴더도 포함한다.
각 세션에 있던 MCAP, 메타데이터, 설정·경로 스냅샷, 제어기 로그와 원본 결과 파일을 그대로 포함한다.
새 결과 집계나 그래프는 추가하지 않았다. 기존 −10°·−5°·0° 업로드는 유지된다.

이 폴더의 모든 압축 조각을 받은 뒤 새 디렉터리에 복구한다.

```bash
sha256sum -c SHA256SUMS
mkdir -p restored
cat original_sessions.tar.zst.part-* | zstd -dc | tar -xf - -C restored
python3 verify_restored.py restored
```
