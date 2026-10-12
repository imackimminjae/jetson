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


@dataclass
class RoadLimits:
    """Evidence thresholds, independent of planner constraints and steering tuning."""
    road_confirmation_sec: float = 3.0
    road_evidence_fresh_sec: float = 0.6
    road_lookahead_m: float = 20.0
    road_corridor_half_width_m: float = 3.0
    road_blocked_span_m: float = 5.0
    road_supported_span_m: float = 2.0
    road_pose_skew_sec: float = 0.25

    def __post_init__(self):
        if any(not math.isfinite(v) or v <= 0 for v in vars(self).values()):
            raise ValueError('all road limits must be finite and positive')
        if self.road_supported_span_m+self.road_blocked_span_m > self.road_lookahead_m:
            raise ValueError('road lookahead must include supported and blocked spans')
        # Bound per-observation work regardless of an accidental parameter change.
        if self.road_lookahead_m > 40 or self.road_corridor_half_width_m > 10:
            raise ValueError('road lookahead/half width must be <= 40/10 m')


class RoadRouteCheck:
    """Bounded grid sampling: classify a route/map mismatch, never physical road ends.

    Unknown/outside cells cannot prove blockage. Only scalar evidence is retained;
    neither this class nor the runner stores decoded grids between observations.
    """

    def __init__(self, points, limits=None):
        self.route = RouteMonitor(points, 0)
        self.limits = limits or RoadLimits()
        self.since = self.last_observed = None
        self.last_id = None
        self.block = None
        self.observations = 0
        self.evidence = {'state': 'unavailable', 'reason': 'no grid evidence'}

    def sample(self, grid, position, yaw, progress, positive_is_drivable=False,
               threshold=0, expected_frame='Vehicle'):
        """Use the pose at grid reception; grid origin orientation is a quaternion."""
        def unavailable(reason):
            return {'state': 'unavailable', 'reason': reason}
        info = grid.info
        if grid.header.frame_id.lstrip('/') != expected_frame.lstrip('/'):
            return unavailable('grid frame mismatch')
        if (not math.isfinite(yaw) or not all(math.isfinite(v) for v in position[:2])
                or not math.isfinite(info.resolution) or info.resolution <= 0
                or info.width <= 0 or info.height <= 0
                or len(grid.data) != info.width*info.height):
            return unavailable('invalid grid or pose')
        origin = info.origin
        q = origin.orientation
        if not all(math.isfinite(v) for v in (origin.position.x, origin.position.y,
                                              q.x, q.y, q.z, q.w)):
            return unavailable('invalid grid origin')
        norm = math.sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w)
        if abs(norm-1.0) > 0.01 or abs(q.x) > 1e-3 or abs(q.y) > 1e-3:
            return unavailable('grid origin must have a planar unit quaternion')
        gy = math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        cy, sy, cg, sg = math.cos(yaw), math.sin(yaw), math.cos(gy), math.sin(gy)

        def cell(x, y):
            dx, dy = x-position[0], y-position[1]
            bx = cy*dx+sy*dy-origin.position.x
            by = -sy*dx+cy*dy-origin.position.y
            ix = math.floor((cg*bx+sg*by)/info.resolution)
            iy = math.floor((-sg*bx+cg*by)/info.resolution)
            if not (0 <= ix < info.width and 0 <= iy < info.height):
                return 'outside'
            value = int(grid.data[iy*info.width+ix])
            if not 0 <= value <= 100:
                return 'unknown'
            return 'free' if ((value > threshold) if positive_is_drivable
                              else (value <= threshold)) else 'blocked'

        limit = self.limits
        half = limit.road_corridor_half_width_m
        offsets = [(-half+2*half*i/math.ceil(2*half/.5))
                   for i in range(math.ceil(2*half/.5)+1)]

        def corridor(arc):
            i = next((j for j in range(len(self.route.lengths))
                      if self.route.arc[j+1] >= arc),len(self.route.lengths)-1)
            a,b = self.route.points[i:i+2];length = self.route.lengths[i]
            u = (arc-self.route.arc[i])/length
            x,y = a[0]+u*(b[0]-a[0]), a[1]+u*(b[1]-a[1])
            nx,ny = -(b[1]-a[1])/length,(b[0]-a[0])/length
            return [cell(x+nx*v,y+ny*v) for v in offsets]

        # Continue observing the SAME map-fixed blockage as ego approaches it.
        # Requiring the earlier free stretch to remain ahead of the front-only BEV
        # would erase evidence precisely when the vehicle reaches the road end.
        # Call expire(now) first: stale seeds cannot excuse an unrelated deviation.
        if self.block is not None:
            start,end = self.block
            samples = [c for k in range(int((end-start)/.5)+1)
                       for c in corridor(start+.5*k)]
            if samples and all(c=='blocked' for c in samples):
                return dict(state='route_unavailable',reason='same previously supported-to-blocked corridor reobserved',
                            blocked_start_m=start,blocked_end_m=end,blocked=len(samples),
                            free=0,unknown=0,outside=0)

        # The front-mounted BEV may start at ego x=0. Observe ego and two front corners.
        vehicle = [cell(position[0]+cy*x-sy*y, position[1]+sy*x+cy*y)
                   for x,y in ((0,0),(.5,-.5),(.5,.5))]
        if any(v != 'free' for v in vehicle):
            return unavailable('vehicle patch not confirmed free')
        end = min(self.route.arc[-1], progress+limit.road_lookahead_m)
        support_start = last_free_arc = blocked_start = None
        count = {'free': 0, 'blocked': 0, 'unknown': 0, 'outside': 0}
        steps = int(max(0,end-progress)/.5)
        for step in range(steps+1):
            arc = progress+.5*step
            cells = corridor(arc)
            for c in cells: count[c] += 1
            if 'free' in cells:
                if blocked_start is not None:
                    support_start = arc
                support_start = arc if support_start is None else support_start
                last_free_arc = arc
                blocked_start = None
            elif all(c == 'blocked' for c in cells):
                if support_start is not None:
                    if blocked_start is None:
                        if last_free_arc-support_start >= limit.road_supported_span_m:
                            blocked_start = arc
                        else:
                            support_start = None
                    if blocked_start is not None and arc-blocked_start >= limit.road_blocked_span_m:
                        return dict(state='route_unavailable',reason='observed supported-to-blocked route corridor',
                                    blocked_start_m=blocked_start,blocked_end_m=arc,**count)
            else:
                # A gap cannot be stitched into a supported-to-blocked transition.
                support_start = blocked_start = None
        state = 'unavailable' if count['unknown'] or count['outside'] else 'supported'
        return dict(state=state,reason='unknown/outside route samples' if state=='unavailable'
                    else 'no confirmed route blockage',**count)

    def expire(self,now):
        if self.last_observed is None or now-self.last_observed > self.limits.road_evidence_fresh_sec:
            self.since = self.block = None
            self.observations = 0

    def observe(self, now, observation_id, evidence):
        if self.last_id is not None and observation_id <= self.last_id:
            return  # Duplicate or backwards stamps do not provide new evidence.
        self.last_id = observation_id
        gap = self.last_observed is None or now-self.last_observed > self.limits.road_evidence_fresh_sec
        self.last_observed = now
        self.evidence = evidence
        if evidence['state'] != 'route_unavailable':
            self.since = self.block = None
            self.observations = 0
            return
        block = (evidence['blocked_start_m'],evidence['blocked_end_m'])
        overlap = self.block is not None and min(block[1],self.block[1])-max(block[0],self.block[0]) >= 1.0
        if gap or not overlap or self.since is None:
            self.since = now
            self.observations = 0
        self.block = block
        self.observations += 1

    def confirmed(self, now):
        return (self.since is not None and self.last_observed is not None
                and now-self.last_observed <= self.limits.road_evidence_fresh_sec
                and self.last_observed-self.since >= self.limits.road_confirmation_sec
                and self.observations >= 3)

    def detail(self, now):
        value = dict(self.evidence)
        if self.last_observed is None or now-self.last_observed > self.limits.road_evidence_fresh_sec:
            value = {'state':'unavailable','reason':'grid evidence missing or stale'}
        value.update(observations=self.observations,confirmed=self.confirmed(now))
        return value
