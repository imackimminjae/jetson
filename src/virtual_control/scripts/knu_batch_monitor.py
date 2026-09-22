"""Deterministic KNU route outcome checks, independent of ROS and wall clock."""

from dataclasses import dataclass
import math


@dataclass
class Limits:
    stop_sec: float = 10.0
    movement_m: float = 0.5
    off_route_m: float = 15.0
    off_route_sec: float = 3.0
    reverse_m: float = 8.0
    reverse_sec: float = 3.0
    no_progress_sec: float = 30.0
    goal_radius_m: float = 8.0
    finish_remaining_m: float = 12.0
    min_progress_ratio: float = 0.85
    route_timeout_sec: float = 900.0

    def __post_init__(self):
        if any(not math.isfinite(v) or v <= 0 for v in vars(self).values()):
            raise ValueError('all route limits must be finite and positive')
        if self.min_progress_ratio > 1:
            raise ValueError('min_progress_ratio must be <= 1')


class RouteMonitor:
    """Follow a local arclength window so crossings cannot jump to the finish."""

    def __init__(self, points, start_time, limits=None):
        self.points = [tuple(p[:2]) for p in points]
        self.limits = limits or Limits()
        self.lengths = [math.dist(a, b) for a, b in zip(self.points, self.points[1:])]
        if (not self.lengths or any(d <= 0 or not math.isfinite(d) for d in self.lengths)
                or any(not math.isfinite(v) for p in self.points for v in p)):
            raise ValueError('route needs finite, distinct consecutive points')
        self.arc = [0.0]
        for length in self.lengths:
            self.arc.append(self.arc[-1] + length)
        self.started = self.last_update = start_time
        self.last_movement = self.last_progress = start_time
        self.movement_anchor = None
        self.previous_position = None
        self.best = self.progress = self.progress_anchor = 0.0
        self.off_since = self.reverse_since = None
        self.cross_track = self.goal_distance = math.inf
        self.outcome = None

    def update(self, now, position):
        if self.outcome:
            return self.outcome
        if now < self.last_update:
            raise ValueError('time must be monotonic')
        if any(not math.isfinite(v) for v in position[:2]):
            return self._finish('invalid_pose')
        p = tuple(position[:2])
        dt = now - self.last_update
        self.last_update = now
        if (self.previous_position is not None and
                math.dist(p, self.previous_position) > max(10.0, 10.0*dt)):
            return self._finish('pose_jump')
        self.previous_position = p
        low = max(0.0, self.progress - 30.0)
        high = min(self.arc[-1], self.progress + max(15.0, 10.0 * dt))
        candidates = []
        for i, (a, b, length) in enumerate(zip(self.points, self.points[1:], self.lengths)):
            if self.arc[i + 1] < low or self.arc[i] > high:
                continue
            u = ((p[0]-a[0])*(b[0]-a[0]) + (p[1]-a[1])*(b[1]-a[1])) / length**2
            u = max(max(0.0, (low-self.arc[i])/length),
                    min(min(1.0, (high-self.arc[i])/length), u))
            q = (a[0]+u*(b[0]-a[0]), a[1]+u*(b[1]-a[1]))
            candidates.append((math.dist(p, q), self.arc[i] + u*length))
        self.cross_track, projected = min(candidates)
        if self.cross_track <= self.limits.off_route_m:
            self.progress = projected
        self.best = max(self.best, self.progress)
        self.goal_distance = math.dist(p, self.points[-1])
        limit = self.limits
        if self.movement_anchor is None:
            self.movement_anchor = p
        elif math.dist(p, self.movement_anchor) >= limit.movement_m:
            self.movement_anchor = p
            self.last_movement = now
        if self.best - self.progress_anchor >= limit.movement_m:
            self.progress_anchor = self.best
            self.last_progress = now
        self.off_since = self._since(self.cross_track > limit.off_route_m, self.off_since, now)
        self.reverse_since = self._since(
            self.best-self.progress > limit.reverse_m, self.reverse_since, now)
        # Current progress, not historical best: reversing to the endpoint cannot pass.
        near_end = (self.progress / self.arc[-1] >= limit.min_progress_ratio and
                    self.arc[-1]-self.progress <= limit.finish_remaining_m)
        if near_end and self.goal_distance <= limit.goal_radius_m:
            return self._finish('completed')
        if self.off_since is not None and now-self.off_since >= limit.off_route_sec:
            return self._finish('off_route')
        if self.reverse_since is not None and now-self.reverse_since >= limit.reverse_sec:
            return self._finish('wrong_direction')
        if now-self.last_movement >= limit.stop_sec:
            return self._finish('stopped_10s')
        if now-self.last_progress >= limit.no_progress_sec:
            return self._finish('no_progress')
        if now-self.started >= limit.route_timeout_sec:
            return self._finish('route_timeout')
        return None

    @staticmethod
    def _since(condition, previous, now):
        return (now if previous is None else previous) if condition else None

    def _finish(self, outcome):
        self.outcome = outcome
        return outcome

    def snapshot(self):
        return {'progress_ratio': self.progress/self.arc[-1],
                'best_progress_ratio': self.best/self.arc[-1],
                'cross_track_m': self.cross_track, 'goal_distance_m': self.goal_distance}
