# Phase 14.8 — Path-Tracking Stability Audit

## 1. Objective
Determine why the ego saturates steering and accumulates large cross-track/heading
error on the Phase 13/14 turn path, isolating cause A–H **off-CARLA first**.

## 2. Baseline Git commit
`1a531a9`, working tree clean, Phase 14.7 traffic variants confirmed reverted.

## 3. Files inspected
`control/purePursuitController.m`, `control/vehicleController.m`,
`control/bicycleModel.m`, `config/vehicleConfig.m`,
`carlaIntegration/matlab/carlaClosedLoopStep.m`,
`carlaIntegration/matlab/carlaGenerateIntersectionTurnPath.m`,
`carlaIntegration/simulink/CarlaClosedLoopBlock.m`.

## 4. Controller architecture (traced)
```
reference path (global frame)
  -> purePursuitController: nearest point, then first point at >= Ld
     Ld = max(3.0, 0.5*v + 2.0)
     alpha = atan2(dy,dx) - yaw ;  steer = atan2(2*L*sin(alpha), Ld)
     clamp +-maxSteerAngle (35 deg)
  -> vehicleController: rate limit |dsteer| <= maxSteerRate*dt (60 deg/s)
     then clamp +-35 deg ; longitudinal P control (Kp = 1.0)
  -> carlaClosedLoopStep: carlaSteer = steerSign * steer / maxSteerAngle,
     steerSign = -1, clamped to [-1,1]
  -> CARLA VehicleControl
```
Conventions: wheelbase 2.7 m, max steer 35 deg, max steer rate 60 deg/s,
positive steering = left = increasing yaw (right-handed project frame).

**Critical structural note:** in the live loop the controller does NOT follow a
fixed path. It follows `pathSmoothing(selectedTrajectory)`, which the planner
**re-selects every tick**. That is the one structural difference between the live
loop and any offline test, and it turns out to be decisive.

## 5. Baseline offline results (fixed reference path)
Same frozen controller, same frozen plant, same parameters, no planner, no CARLA:

| Metric | CARLA (14.7) | Offline fixed path |
|---|---|---|
| mean CTE | 1.90 m | **0.031 m** |
| RMS CTE | – | 0.059 m |
| max CTE | 2.52 m | **0.196 m** |
| mean heading err | 12.9 deg | **0.49 deg** |
| max heading err | 49.3 deg | **3.86 deg** |
| steering saturation | **69 %** | **0.0 %** |
| steering-rate saturation | pinned | **0.0 %** |
| time outside 1.0 m | – | 0.0 % |
| completion | no | **yes** (final pos err 2.83 m, head err 0.1 deg) |

Evidence: `results/figures/phase148_baseline_tracking.png`

## 6. Steering sign / frame verification (section 7)
All seven numeric assertions pass:

| Test | Result |
|---|---|
| left of path -> right steer | PASS (-0.2889) |
| right of path -> left steer | PASS (+0.2889) |
| aligned -> ~zero | PASS (0.000000) |
| yawed left -> right steer | PASS (-0.3538) |
| yawed right -> left steer | PASS (+0.3538) |
| plant: +steer -> +yaw | PASS (+0.0196) |
| CARLA map: +left -> negative raw steer | PASS (-0.2857) |

**Cause B (sign/frame) is eliminated.**

## 7. Reference-switching experiment (section 12 — the decisive test)
`localPlanner.m` spreads 15 candidates over +-2.5 m. Emulating the planner
re-selecting among those offsets, with the controller untouched:

| switch rate | mean CTE | max CTE | max heading | steer sat | rate sat |
|---|---|---|---|---|---|
| 0.00 | 0.031 m | 0.196 m | 3.9 deg | 0.0 % | 0.0 % |
| 0.10 | 0.822 m | 2.411 m | 33.8 deg | 3.8 % | 43.9 % |
| **0.24** (measured live rate) | **1.155 m** | **3.281 m** | **60.9 deg** | 17.3 % | **59.8 %** |
| 0.45 | 0.776 m | 2.149 m | 36.9 deg | 4.2 % | **68.2 %** |

Reference switching **alone** reproduces the live pathology — CTE of the right
order, heading error exceeding 60 deg, and rate-limit pinning rising to 68 %.
The controller is faithfully chasing a target that keeps moving.

## 8. Lookahead sweep (section 8)
`results/figures/phase148_lookahead_sweep.csv`

| config | switch 0.00 mean CTE | switch 0.24 mean CTE | 0.24 max head | 0.24 steer sat |
|---|---|---|---|---|
| fixed 3 m | 0.018 m | 2.353 m | 84.3 deg | 44.8 % |
| production (~3.25 m) | 0.031 m | 1.155 m | 60.9 deg | 17.3 % |
| fixed 4 m | 0.031 m | 1.227 m | 65.4 deg | 16.7 % |
| fixed 5 m | 0.052 m | 0.694 m | 33.1 deg | **0.0 %** |
| fixed 6 m | 0.079 m | 0.525 m | 23.7 deg | **0.0 %** |
| fixed 7 m | 0.118 m | **0.410 m** | **18.2 deg** | **0.0 %** |

On a stable reference every configuration is fine, and shorter is marginally
better (classic corner-cutting trade-off, 12 cm spread). Under realistic
switching the ordering inverts: the production lookahead is too short to be
robust, and 5-7 m removes steering saturation entirely.

## 9-10. Not performed
Steering-rate sweep and speed/curvature sweep were not run. With the fixed-path
baseline already at 0 % rate saturation, neither can explain the live behaviour;
the switching experiment already localises the cause.

## 11. Path geometry
88.5 m, 90 waypoints, min radius 12.0 m, tracked offline to 0.196 m max error —
the geometry is trackable. **Cause E eliminated.**

## 12. Root cause determination
**Cause H (combination), dominated by planner reference switching — not a
controller defect.**

1. Controller + plant + path in isolation: essentially perfect (0.031 m, 0 %
   saturation). Causes A/B/C/D/E/F/G individually eliminated as primary.
2. Injecting reference switching at the measured live rate, changing nothing
   else, reproduces the live failure signature.
3. Contributing factor: the production lookahead (~3.25 m at these speeds) is
   short enough that switching is passed straight through to the steering
   command rather than filtered.

Per section 12 this phase **stops here** — candidate switching is the primary
cause, so controller tuning is not the fix and the planner is not to be modified
in this phase.

## 13-15. No production change made
No file in `control/`, `planning/`, `perception/`, `prediction/`, `decision/`, or
the traffic logic was modified. Only two new test harnesses were added. A
lookahead change is **recommended but not applied**: section 14 generalization
(left/right/gentle/tight/varied initial error) was not run, and I will not change
a production parameter on single-path evidence.

## 16-18. CARLA validation — not performed
Deliberate. The finding is that the fix belongs at the planner/reference-stability
level, which section 12 forbids changing here. Re-running CARLA without a change
would only re-measure Phase 14.7.

## 19. Regression
Not required — no production code changed.

## 20. Remaining limitations
- Generalization across path variants not tested.
- Offline plant is the kinematic bicycle model; CARLA dynamics, dt variance
  (measured up to 5.5 s outliers) and ego-state noise are additional live factors
  not reproduced here, so the switching experiment shows sufficiency, not
  exclusivity.
- Zero-collision remains unmet.

## 21. Git
Baseline `1a531a9`. Added: `tests/testPhase148ControllerOffline.m`,
`tests/testPhase148SignAndSwitching.m`, this report, and evidence files. No
production code modified.

## 22. Final verdict

**CONDITIONAL PASS**

The controller is proven stable offline (0.031 m mean CTE, 0 % steering
saturation, full completion) and free of sign/frame defects, and the live
instability is reproduced by reference switching alone. CARLA validation is
deliberately incomplete because the indicated fix lies outside this phase's
permitted scope.

**Recommended next phase:** reduce planner reference switching (e.g. examine
`adaptivePlanner`'s consistency cost under a 39 % K2-fallback regime, where the
fallback path may bypass the consistency term), and only then re-evaluate
lookahead with generalization testing. Do not tune the controller first.
