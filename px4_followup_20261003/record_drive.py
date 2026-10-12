#!/usr/bin/env python3
"""Record/download a user-driven SIH run. Never sends motion, ARM or parameter commands."""
import argparse
import fcntl
import json
import os
from pathlib import Path
import re
import select
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parent
WS = ROOT.parent


def discover_proxy():
    matches = []
    for p in Path('/proc').glob('[0-9]*'):
        try:
            args = (p / 'cmdline').read_bytes().decode().split('\0')
            if not any(Path(a).name == 'mavproxy.py' for a in args):
                continue
            if '--out=127.0.0.1:14540' not in args:
                continue
            ports = {}
            for line in (p / 'net/udp').read_text().splitlines()[1:]:
                fields = line.split()
                ports[fields[9]] = int(fields[1].split(':')[1], 16)
            sockets = []
            for fd in sorted((p / 'fd').iterdir(), key=lambda x: int(x.name)):
                target = os.readlink(fd)
                if target.startswith('socket:['):
                    port = ports.get(target[8:-1])
                    if port and port not in (14540, 14541, 14550):
                        sockets.append(port)
            if sockets:
                tlog = (p / 'cwd').resolve() / 'mav.tlog'
                if tlog.exists():
                    matches.append((int(p.name), sockets[0], tlog))
        except (OSError, ValueError, IndexError):
            continue
    if len(matches) != 1:
        raise RuntimeError('기존 MAVProxy 연결을 하나로 확인할 수 없습니다. 연결 상태를 알려주세요.')
    return matches[0]


def query(port, tlog, group, output):
    result = subprocess.run([
        sys.executable, str(ROOT/'read_board.py'), '--proxy-port', str(port),
        '--tlog', str(tlog), '--group', group,
    ], capture_output=True, text=True, timeout=120)
    text = result.stdout + result.stderr
    (output / (group+'.txt')).write_text(text)
    if result.returncode:
        raise RuntimeError(text)
    return text


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true', help='연결·로거 상태만 확인; 기록 시작 안 함')
    parser.add_argument('--seconds', type=int, default=300, help='최대 기록 시간 (기본 300초)')
    args = parser.parse_args()
    if not 10 <= args.seconds <= 600:
        parser.error('--seconds는 10~600초 범위여야 합니다.')
    lock = (ROOT/'recorder.lock').open('w')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    pid, port, tlog = discover_proxy()
    output = ROOT / time.strftime('record_%Y%m%d_%H%M%S')
    output.mkdir(exist_ok=False)
    print(f'기존 보드 연결 확인 중 (MAVProxy {pid})…', flush=True)
    status = query(port, tlog, 'off_status', output)
    if '[logger]' not in status:
        raise RuntimeError('보드 응답을 확인하지 못했습니다. 기록을 시작하지 않습니다.')
    if args.check:
        print(status)
        return
    if 'Not logging' not in status:
        raise RuntimeError('이미 기록 중이거나 로거 상태가 불명확합니다. 기존 기록을 건드리지 않습니다.')
    owned = False
    remote = None
    stopped = False
    try:
        owned = True  # Also stop if start command succeeded but response parsing failed.
        text = query(port, tlog, 'record_start', output)
        match = re.search(r'Log file: (/fs/microsd/log/[^\s]+\.ulg)', text)
        if 'Full File Logging Running' not in text or not match:
            raise RuntimeError('기록 시작 확인 실패: '+text)
        remote = match.group(1)
        print('기록 중입니다. 평소 방식으로 직접 주행하세요.', flush=True)
        print('정지 → 주행 → 입력 0으로 감속 → 완전 정지 후 30초 대기.', flush=True)
        print(f'끝나면 이 창에서 Enter. 최대 {args.seconds}초 후 기록만 자동 종료합니다.', flush=True)
        print('이 프로그램은 차량을 움직이거나 정지시키지 않습니다.', flush=True)
        deadline = time.monotonic()+args.seconds
        while time.monotonic() < deadline:
            ready, _, _ = select.select([sys.stdin], [], [], max(0, min(1, deadline-time.monotonic())))
            if ready:
                sys.stdin.readline()
                break
    except KeyboardInterrupt:
        print('\n기록을 종료합니다. 차량 제어에는 개입하지 않습니다.', flush=True)
    finally:
        if owned:
            text = query(port, tlog, 'record_stop', output)
            stopped = 'Not logging' in text
            if not stopped:
                print('기록 종료 응답을 확인하지 못했습니다. QGC에서 logger status를 확인하세요.', flush=True)
    if not remote or not stopped:
        raise RuntimeError('완료된 로그를 확인할 수 없어 자동 다운로드를 중지했습니다.')
    local = output/'drive.ulg'
    print('기록 종료 확인. 보드 로그를 가져옵니다…', flush=True)
    subprocess.run([
        sys.executable, str(WS/'px4_rest_audit_20261002/download_via_proxy.py'),
        '--proxy-port', str(port), '--tlog', str(tlog), '--remote', remote, '--local', str(local),
    ], check=True, timeout=1000)
    env = dict(os.environ, MPLCONFIGDIR='/tmp/px4-followup-mpl')
    subprocess.run([
        str(WS/'px4_rest_audit_20261002/.venv/bin/python'),
        str(ROOT/'analyze_ulog.py'), str(local), '--output', str(output/'analysis'),
    ], check=True, env=env)
    summary = json.loads((output/'analysis/summary.json').read_text())
    if summary.get('missing'):
        print('저장은 완료했지만 필요한 내부 토픽이 부족합니다: '+', '.join(summary['missing']), flush=True)
        print('정지 상태에서 새 입력 발행이 없으면 actuator_outputs_sim이 기록되지 않을 수 있습니다.', flush=True)
    print(f'저장·기본 분석 결과: {output}', flush=True)


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(f'중단: {e}', file=sys.stderr)
        sys.exit(1)
