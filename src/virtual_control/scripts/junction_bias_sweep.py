#!/usr/bin/env python3
"""Select the short junction missions with their frozen waypoint bias windows."""
import argparse
import json
import os
from pathlib import Path

WORKSPACE = Path(__file__).resolve().parents[3]
PROFILE = WORKSPACE/'src/virtual_control/data/knu_routes/knu_junction_profile.json'


def main():
    profile = json.loads(PROFILE.read_text())
    definitions = {int(m['scenario'].removeprefix('scenario')): m
                   for m in profile['definitions']}
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('scenario', type=int, nargs='*', metavar='SCENARIO',
                        help='S26–S31 중 선택; 생략하면 6개 전체')
    parser.add_argument('--bias', type=int, nargs='+', choices=[-10, -5, 0, 5, 10],
                        default=[0, -5, 5])
    parser.add_argument('--runs', type=int, default=1)
    parser.add_argument('--order', choices=['bias', 'scenario'], default='bias')
    parser.add_argument('--prepare-only', action='store_true')
    args = parser.parse_args()
    scenarios = args.scenario or sorted(definitions)
    if any(n not in definitions for n in scenarios):
        parser.error('시나리오는 26~31 중에서 선택하세요')
    command = ['bash', str(WORKSPACE/'run_scenario_bias.sh')]
    command += [str(n) for n in scenarios]
    command += ['--bias', *map(str, args.bias), '--runs', str(args.runs), '--order', args.order]
    for number in scenarios:
        start, end = definitions[number]['bias_waypoints']
        command += ['--bias-waypoints', f'{number}:{start}:{end}']
    if args.prepare_only:
        command.append('--prepare-only')
    os.execv('/bin/bash', command)


if __name__ == '__main__':
    main()
