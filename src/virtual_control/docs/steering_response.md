# Steering response compensation

## Current status (22:23 SIH recording)

A new profile uses delay=0.30 s, tau=1.00 s and N=30, reidentified against SIH
**ground truth**, with a held-out part of the latest oscillating run. It also
aligns the reference preview with measured travel. See [sih_sway_tuning.md](sih_sway_tuning.md)
for the current evidence, configuration, and validation limits.

## Historical trial: withdrawn after HILS regression

The user reported substantially worse path departure in HILS after this trial.
At that point, the split profile was restored to actuator_model=false, 20 prediction
steps, and dormant delay/time-constant values of 0.20/0.85 s. The 6 m/s target
remains a separate user-requested change. The offline fit and straight-line
simulation below did not validate the closed-loop response on the actual route
at 6 m/s; they must not be treated as proof of improved HILS tracking.

## Withdrawn experimental configuration

The experimental split controller profile enabled the existing lower MPC steering-state model
with a 0.30 s command delay and a 1.10 s first-order time constant. At the configured
10 Hz control/prediction rate, the delay queue contains three commands. The MPC
predicts heading from the integral of effective steering over each interval;
effective steering and pending commands also carry over between control cycles.

The upper and lower target speeds remain 6.0 m/s. The lower prediction increases
from 20 to 30 steps at 0.10 s (three seconds, or 18 m at 6 m/s). This fits within
the nominal 24 m upper path at the current grid extent and planning speed.
The existing command calibration remains 0.30 rad per unit
normalized steering. The output filter stays disabled: an additional filter would
introduce dynamics beyond those currently represented in the prediction.

## Evidence and parameter selection

The local `mav.tlog` from 2026-09-23 contains transmitted `COMMAND_LONG` command 187
(steering in param2), `ATTITUDE.yawspeed`, and `LOCAL_POSITION_NED` planar velocity.
The associated latest controller logs are
`upper_planner_node_12960_1790166120521.log` and
`lower_tracking_mpc_node_12998_1790166121676.log` under `~/.ros/log`.

Fit the effective steering inferred from `atan(2.8 * yaw_rate / planar_speed)`
against the delayed first-order response to the recorded normalized command.
Both signals use the NED steering/yaw sign convention. Commands are held between
samples, velocity and yaw rate are interpolated to 0.02 s, and samples below
1 m/s are excluded. This is an equivalent steering/yaw response model, not a
measurement of the physical steering actuator angle or its delay in isolation.

The latest run uses a 3 m/s target. Fitting uses 21:22:11–21:22:40 KST;
21:22:40–21:23:06 is held out. The filter is initialized at 21:22:08 with the
first inferred steering value; the first three seconds are excluded from the
fit. With command gain fixed
at 0.30 rad, a 0.30 s delay gives a fitted time constant of 1.081 s, rounded to
1.10 s for the profile.

| Model | Fit RMS error (rad) | Held-out RMS error (rad) |
| --- | ---: | ---: |
| Instantaneous steering | 0.12743 | 0.21452 |
| Dormant 0.20 s delay / 0.85 s time constant | 0.02805 | 0.04217 |
| Enabled 0.30 s delay / 1.10 s time constant | 0.01501 | 0.01677 |

The approximately one-second time from command sign reversal to yaw-rate sign
reversal combines transport delay and vehicle response. It is not used as a
one-second pure command delay. The earlier 1.5 m/s run fits a faster response,
so these constants are an approximation and do not establish speed-independent
vehicle dynamics. No 6 m/s HILS measurement was available for this calibration.

## Verification and limits

`LowerMpc.DelayedNonlinearVehicleRecoversAtRequestedSpeeds` exercises the real MPC
against an independent 200 Hz nonlinear bicycle simulation at 1.5, 3.0, and
6.0 m/s. It starts 0.5 m off a straight path with 0.05 rad heading error and
checks command limits, bounded deviation, and recovery over 20 seconds. It also
varies the plant delay/time constant around the configured model.

With only 20 prediction steps, the 1.5 m/s case retained 0.138 m of lateral
error after 20 seconds even with matched response parameters. A slower plant
(0.40 s delay, 1.30 s time constant) retained 0.230 m and 0.161 rad of heading
error. This motivated extending the prediction to 30 steps instead of changing
the error weights or adding an output filter.

With 30 steps, all nine speed/response cases pass: lateral error stays below
0.9 m and finishes below 0.05 m, with final heading error below 0.02 rad.
The complete lower MPC and actuator suites pass all 15 tests. An isolated
startup of the installed profile also confirms N=30, actuator_model=1,
delay=0.30 s and tau=1.10 s with the vehicle output backend disabled.

This test does not establish performance on changing curved paths, Isaac road
geometry, or actual hardware. The upper planner's infeasible-corridor failures
and the lower controller's stale-path neutral policy are separate from steering
response compensation. Validate the new model on HILS by comparing commanded
steering, measured yaw rate, lateral error, and path validity.
