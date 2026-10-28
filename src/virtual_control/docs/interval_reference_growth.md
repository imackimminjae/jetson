# Optional upward growth limit for interval reference B

`preview_interval_reference_growth_rate_mps` defaults to `0.0`. Zero preserves
legacy B updates: initialize from the smallest eligible observed width, accept
all decreases and increases up to `1.5*B`, and reject larger increases.

A positive value enables a different B update:

    cap = preview_interval_reference_growth_rate_mps / upper_planner_rate_hz
    Bnew = candidate                              if B is uninitialized
    Bnew = min(candidate, Bold + cap)             otherwise
    Bnew = Bold                                   when no candidate is eligible

Candidate eligibility and selection remain unchanged: a unique containing
interval must have both boundaries observed, a finite positive original length,
and no nominal fallback. The smallest eligible width across the usable preview,
including k0, supplies the candidate. Narrowing remains immediate. No additional
B history, timestamp, width floor, or corridor identity is introduced.

Unlike the legacy growth gate, this update gradually recovers even when a
persistent widened road exceeds `1.5*B`. The factor `1.5` remains the unchanged
ON/OFF treatment threshold. All raw road observations, interval treatment
formulas, solver costs, and vehicle constraints retain their existing meaning.
The recorded B candidate continues to report the original observed width even
when its effect on B is limited.

The rate uses the configured nominal planning period, not measured elapsed time.
For example, 1 m/s at 10 Hz allows 0.1 m growth per candidate-bearing update.
Missed callbacks or missing evidence slow wall-clock growth; the code does not
catch up after a gap. This intentionally keeps B as the sole state. Initialization
and resets preserve the existing behavior.

The limit targets changes in which preview section supplies B. In recorded S1
runs, near-road width around 11.8 m briefly lost an observed boundary, moving B
support to wider downstream sections around 14 m. A newly complete 20 m interval
then crossed into ON treatment and contracted roughly 6 m per side, exceeding
first-step lateral reach. A 1 m/s upward limit keeps that interval OFF in frozen
log replay, retaining the existing 2 m inset. This is a feasibility diagnosis,
not evidence that closed-loop driving will improve.

Delayed growth is appropriate to evaluate because B controls additional
centering contraction rather than defining the raw drivable road. A true wider
road still becomes eligible immediately as raw geometry, while B catches up
progressively. However, OFF treatment can contract more than ON for narrow
intervals, so verify processed bounds, feasibility, completion, heading churn,
body yaw, and measured clearance in live comparisons. The limit does not enforce
a vehicle-footprint clearance or eliminate every threshold crossing.

Startup logs report the rate and derived per-update cap. The interval-state
message and solver trace formats remain unchanged. Tests cover the exact legacy
sequence, immediate narrowing and recovery after a spike, persistent widening,
missing/invalid evidence, invalid cap values, and the recorded long-interval
contraction sequence. The initial experiment should set the new parameter only
in its isolated configuration; active defaults remain zero.

In the 2026-10-16 exploratory closed-loop screens with previous-plan heading
weight 1, a 1 m/s B growth limit improved some plan-jump and feasibility metrics
but increased body-yaw variation on S2 and S3. A 2 m/s S2 screen also worsened
the early-junction body response and introduced a local infeasible plan. These
were single-run comparisons, not a general rejection of every B setting, but
the frozen-input feasibility improvement alone did not justify enabling the
feature in a recommended driving profile.

## Optional interval-treatment transition (2026-10-16)

`preview_interval_transition_relative_band` defaults to `0.0`, preserving the
legacy hard ON/OFF inset exactly. A positive band blends the inset with cubic
smoothstep as `L/B` decreases from `1.5` to `1.5 - band`. The tested candidate
uses `0.25` (OFF at/above 1.5, full ON at/below 1.25). This parameter does not
change B updates. Nominal fallback and unavailable-B treatment are unchanged.
The existing diagnostic ON flag still records the legacy threshold classification;
the actual processed length, not that flag alone, describes the blended constraint.
Raw intervals, candidate extraction and vehicle constraints are not changed, but
the resulting processed geometry can affect branch continuity and the selected plan.
Values must be finite and in `[0, 1.5)`. No new persistent state is introduced.

The frozen-input audit is in `followup_20261016/offline_b_soft/`. This is an
experimental option, not a default profile change or proof of closed-loop benefit.
