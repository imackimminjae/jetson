"""Validate operator-facing waypoint windows before preparing any run."""
from knu_global_path_publisher import load_route


def parse_bias_windows(specifications, scenarios, routes_directory, prefixes=None):
    if not specifications:
        return {}
    prefixes = prefixes or {}
    result = {}
    for text in specifications:
        try:
            values = list(map(int, text.split(':')))
        except ValueError as error:
            raise ValueError('bias window must be START:END or SCENARIO:START:END') from error
        if len(values) == 2:
            names, bounds = scenarios, values
        elif len(values) == 3:
            names, bounds = [f'scenario{values[0]}'], values[1:]
        else:
            raise ValueError('bias window must be START:END or SCENARIO:START:END')
        start, end = bounds
        for name in names:
            if name not in scenarios:
                raise ValueError(f'{name}: select the scenario before specifying its bias window')
            if name in result:
                raise ValueError(f'{name}: specify only one bias window')
            points = load_route(name, routes_directory, prefixes.get(name, 0))
            if not 1 <= start <= end <= len(points)-1:
                raise ValueError(f'{name}: wp0 window must satisfy 1 <= START <= END <= {len(points)-1}')
            result[name] = [start, end]
    missing = set(scenarios)-result.keys()
    if missing:
        raise ValueError('Specify a bias window for every selected scenario: '+', '.join(sorted(missing)))
    return result
