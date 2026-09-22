# SIH 2026 — Problem Statement 26037
## Adaptive Path Planning and Collision Avoidance for Autonomous Vehicles on Unstructured Indian Roads

## Archived — CARLA integration (Phases 9–15)

A CARLA/MATLAB/Simulink integration layer (`carlaIntegration/`, CARLA-only
configs, a hero urban scene, closed-loop turning, and forensic
collision-recovery work) was built and live-verified against a real CARLA
0.9.16 server across Phases 9–15, entirely alongside — and never modifying
— the MATLAB-only autonomy stack below. The project has since moved to a
**MATLAB/Simulink-only** scope for the current evaluation/release; the
CARLA integration code has been removed from the active tree and its
reports/evidence moved to [docs/archive/carla/](docs/archive/carla/) for
historical reference (full history remains in Git). The MATLAB-only system
(`main.m`, `demo/runDemo.m`, the five scenarios, K1, K2) has no CARLA
dependency and needs no CARLA server, Python bridge, or Unreal Engine to
run.

## Hardening — villageRoad avoid↔brake oscillation

Root-caused, not a dwell/threshold tuning issue: `behaviorDecision.m`'s
relaxation re-test (`RECOVERY_MARGIN`) compounds with `VRU_TTC_FACTOR` for
vulnerable-class agents (`3.0 × 1.5 × 1.5 = 6.75s`), so the relaxation check
itself could return a *more severe* command than the state it was testing
whether to leave. `decisionStateMachine` can't distinguish a relaxation
check from a fresh evaluation, so it read this as a genuine escalation and
applied it immediately, bypassing the dwell time relaxation is supposed to
have. Confirmed with a byte-for-byte debug copy of the real function
(`real=brake debugCopy=brake`), not inferred.

Fix: the relaxation re-test's only valid answers are now "relax to this" or
"hold the current state" — it can never be read as authorizing a step up
from the state it was testing. True escalations (from the raw, un-widened
check) remain immediate and untouched.

**Measured, same seed, real `main.m` runs (not a diagnostic harness) for
both before and after:**

| Metric | Before | After |
|---|---|---|
| Goal reached | Yes | Yes |
| Completion time | 43.0s | **37.5s** |
| Geometric collision | No | No |
| Min. clearance | 2.13m | 2.16m |
| Min. TTC | Inf | Inf |
| Fallback count | 0 | 0 |
| State transitions (total) | 33 | 25 |
| **avoid↔brake transitions** | **19** | **13** |
| Path smoothness | 1.76 rad | 1.91 rad |
| Max steering | 24.5° | 24.8° |
| Max braking | 5.75 | 5.80 |

Oscillation reduced (19→13, -32%), not eliminated — the remaining 13 are
now genuine boundary crossings at the *correct* threshold (confirmed: ttc
≈4.1–4.5s, matching the real margin=1.0 VRU threshold, not the inflated
6.75s artifact), for a slow-moving vulnerable agent (pedestrian, then
later the animal) alongside the road for an extended stretch — a real,
borderline situation, not a bug. Goal completion got *faster* (43.0→37.5s)
and clearance held steady, so the fix did not trade safety or
responsiveness for stability. Path smoothness and max braking both ticked
up very slightly (1.76→1.91 rad, 5.75→5.80) — a minor, honest trade-off
from the different resulting speed profile, not a regression in any of the
success criteria.

## Phase 10 — perception hardening: post-fusion deduplication

Fixed the known duplicate sensor-fusion/tracking bug (root-caused in an
earlier session: villageRoad t=2.00s, a radar detection of the parked car
missed the 2.5m camera-anchor gate by 0.235m and survived as a second
`unknown`-class phantom track). Added a **conservative post-fusion
deduplication pass** in `perception/sensorFusion.m` - the 2.5m association
gate itself is untouched, exactly as instructed. Merges two fused entries
only when: position distance < 3.5m (derived from the actual noise model:
combined camera+radar position-noise std is 1.265m, and 3.5m sits between
its 95%/99% 2D confidence radii of 3.10m/3.84m - comfortably above the
confirmed 2.735m failure, comfortably below the ~10m+ separation between
any two real distinct agents in every scenario inspected) AND class-compatible
(never merges two different confident non-"unknown" classes) AND
velocity-consistent when both sides carry meaningful velocity.

**Verified, not assumed:**
- villageRoad t=2.00s reproduced exactly: 11 pre-dedup entries → 7 post-dedup,
  matching the true 7-agent count exactly. The parked car's duplicate
  (previously id=30) merged into the correctly-classed `car` entry.
- All four suspected duplicate pairs (parked vehicle, pushcart, pedestrian,
  motorcycle) were confirmed merged in the same run - the motorcycle pair
  at 3.39m, the tightest margin observed.
- **One genuine false merge found and reported, not hidden**: in
  `urbanIntersection`, a car and a motorcycle - genuinely different real
  objects - merged for one tick at their 3.40m closest approach near the
  junction (1 of 46 merge events there; 0 of 92 in `marketArea`). The true
  motorcycle-duplicate distance (3.39m) and this false case (3.40m) are
  practically identical, so the distance threshold alone cannot separate
  them - flagged as a specific, scoped item for a future pass, not fixed here.
- Five-scenario regression: all still reach goal, zero geometric collisions
  before and after. Fallback (no-safe-candidate) counts dropped exactly
  where duplicates were the cause - villageRoad 4→0, marketArea 1→0,
  urbanIntersection 6→4 - and stayed flat in highwayMerge/cattleCrossing,
  whose fallback counts were never duplicate-driven. Minor honest
  trade-offs: villageRoad and urbanIntersection completion times each
  slowed by under 2s, and highwayMerge's minimum clearance shifted slightly
  (2.26m→2.09m) - both consistent with the fix legitimately changing
  perception output from the very first tick, cascading into marginally
  different trajectories, not a new safety failure (still zero collisions).

Status: **Phase 6 — Kalman-filter tracking + behavior-aware prediction.**
Building on Phase 5's full perception-to-control loop:
`perception/objectTracking.m` now runs a real constant-velocity Kalman
filter per track over `[x, y, vx, vy]` (position-only measurement, matching
the project brief's tracking diagram and its "Kalman-filter preferable"
guidance) instead of ad-hoc velocity smoothing. `prediction/` gained
`classifyBehavior.m`, which classifies each agent's current motion as
`stopped`/`normal`/`crossing`/`merging` relative to the ego's heading;
`trajectoryPrediction` now uses this alongside `agent.class` to decide
uncertainty growth - so a *car* caught crossing or merging into the ego's
path gets the same heightened-uncertainty treatment a pedestrian doing the
same thing already got, instead of being trusted just because it's a
structured class.

That change alone measurably improved `urbanIntersection` (its three
dynamic structured-class agents were literally crossing/merging through the
junction): clearance 1.72m→2.30m, TTC 2.90s→3.70s. But it also exposed a
real, pre-existing architectural gap: `highwayMerge`'s clearance got much
worse (4.86m→1.17m) from wild steering oscillation - `adaptivePlanner` had
always picked whichever candidate scored best *that instant* with zero
preference for continuity, and near an obstacle several candidates were
close enough in cost that the pick flickered every step (candidate index
bouncing 2→3→2→2→3, later 5→8→8→8→8→4→4→6→8), swinging the vehicle's
heading by 40+ degrees step to step. Fixed with a small consistency cost in
`adaptivePlanner` against the previously-selected candidate index - a
genuinely large safety difference still overrides it.

**Final result, all five scenarios, real noisy perception:** all reach
goal, and for the first time **every scenario holds 2.0m+ clearance**
(2.05m-2.99m) - including `highwayMerge`, which finished 6 seconds *faster*
once it stopped dancing between candidates.

Every function still keeps the fixed signature from `docs/architecture.md`
(see its "Interface change log" for every signature that changed once a
function actually had to do its documented job) so later phases can fill in
real logic without breaking callers.

## Phase 9 — control + closed-loop integration: steering-rate limiting

Inspected the controller/vehicle-model integration before changing anything:
items 1 (planner→controller connection), 2-3 (decision-state→speed
mapping for all 8 real states), 5-6 (bounded steering/accel commands) were
already implemented from prior phases. Note: the task brief referenced a
"yield" state that no longer exists post-Phase-6 (superseded by the 8-state
set) - mapped to the real states instead of inventing a "yield" behavior.

Item 7 ("prevent controller instability when the selected candidate
changes") was a real, measured gap: `adaptivePlanner`'s consistency cost is
soft, not a hard constraint, so the selected candidate index can still jump
substantially between ticks. Measured before touching anything: steering
commands swung 343-700 deg/s at candidate-index jumps (e.g.
`urbanIntersection` candidate 15→1, a 70-degree swing in one 0.1s tick),
and 4 of 5 scenarios repeatedly saturated the absolute ±35° limit - no real
steering rack moves anywhere near that fast. Fixed with a rate limiter in
`control/vehicleController.m` (new `vehicleConfig.maxSteerRate`, 60°/s)
against `egoState.steering`, the angle actually applied last tick.

**Verified: max steering jump dropped to exactly 60°/s (the limit) across
all five scenarios, with zero geometric collisions in any of them** (a new,
distinct measurement — `minClearance` against ground-truth agent positions
with a 1.1m contact threshold — separate from `adaptivePlanner`'s "no safe
candidate" fallback flag, which is a risk signal, not proof of collision;
conflating the two would have overstated risk in every prior phase's
reporting).

**One regression, reported rather than hidden:** `highwayMerge`'s minimum
clearance dropped from 2.73m to 2.26m — still safe, no collision, comfortably
above the 1.1m contact distance — but worse than before. Rate-limiting
steering means the overtake maneuver can't snap to its target as fast,
making the transient pass a bit tighter. This was not tuned away; 60°/s is
a defensible real-world steering-rack rate, and adjusting it further right
now would be exactly the "change parameters to make a scenario pass"
pattern that wasn't asked for.

`main.m`'s per-step CSV log (`results/logs/<scenario>_steps.csv`) now also
records steering command, throttle, brake, and per-step clearance; the
end-of-run summary adds goal-reached, geometric-collision flag, state
transition count, speed-tracking error (mean/max), and max steering/accel/
braking commands actually issued.

## Phase 8 — decision logic: 8-state behavior machine + safety priority hierarchy

`decision/` now implements the brief's full state set — `cruise`, `follow`,
`merge`, `replan`, `avoid`, `wait`, `brake`, `emergency_stop` — with
`behaviorDecision` evaluating the safety priority hierarchy in strict order
(emergency → pedestrian/animal → dynamic obstacle → merge/interaction →
normal navigation) and `decisionStateMachine` walking the brief's
CRUISE→AVOID→BRAKE→REPLAN→CRUISE recovery cycle. Vulnerable road users
(pedestrian/animal/bicycle) get 1.5× the TTC threshold and a wider
proximity radius than vehicles — that tier is the whole point of ranking
their safety above generic obstacle avoidance. New
`decision/behaviorSeverity.m` is the single source of truth for state
ranking, kept monotonic with the speed factors.

Stateflow is installed and licensed here, but charts run inside Simulink
models — driving one from this per-tick MATLAB loop would need a `sim()`
call every 0.1s step. The brief explicitly allows equivalent MATLAB logic
first, which is what this is; see `docs/architecture.md` for the full
reasoning and for the two bugs found during validation (recovery overshoot
causing a `brake<->replan` oscillation, and `wait` sitting on the recovery
ladder at a dead stop).

**Validated across all five scenarios — every clearance improved or held
vs. the previous baseline:**

| Scenario | Clearance before → after | Goal time |
|---|---|---|
| `villageRoad` | 2.05m → **2.16m** | 30.1s → 44.6s |
| `urbanIntersection` | 2.30m → **3.55m** | 16.7s → 28.0s |
| `highwayMerge` | 2.38m → **2.73m** | 23.8s → 30.9s |
| `marketArea` | 2.10m → **2.12m** | 23.1s → 30.8s |
| `cattleCrossing` | 2.99m → **2.99m** | 20.0s → 25.6s |

The efficiency cost is real and large (runs are 25–48% slower) — that is the
brief's explicit "safety should dominate route efficiency" trade, but it is
a trade, and the `decisionSpeedFactors` in `main.m` are where to revisit it.
One caveat worth knowing: `wait` is implemented and reachable but not
exercised by any of the five scenarios, since it requires the ego to be
nearly stopped with a vulnerable road user actively crossing ahead.

## Phase 7 — adaptive planning: full cost function, replanning instrumentation, path smoothness

`adaptivePlanner`'s cost function now uses all six documented weights
(`obstacleClearance` and `uncertainty` were genuinely new; `speedChange` is
a curvature-implied-speed proxy, since candidates only vary laterally in
this architecture - speed is handled by the decision layer separately, not
per-candidate. See the function's doc comment for the full reasoning).
Re-verified against all five scenarios: identical clearance numbers to
before, so the added terms refine decisions without disrupting the
validated baseline.

Added replan-trigger/latency instrumentation (`main.m`) and implemented
`evaluation/calculatePathSmoothness.m` (on the ego's actual driven path,
not just one candidate) and `evaluation/calculateReplanningLatency.m`.
Two honest findings, not bugs, worth knowing:

- **Measured latency comes back ~0.00s.** This simulation replans every
  tick unconditionally with no artificial computation delay, so once a
  trigger fires, the same tick's plan already reflects it - there's no
  pipeline lag to measure the way a real deployed system would have. See
  `docs/architecture.md`'s "Replanning trigger/latency instrumentation"
  section for the full reasoning, including why ID-based triggers
  (`onNewObstacle`/`onTrackLost`) were tried and dropped in favor of the
  TTC-based one.
- **That investigation surfaced a real perception-layer bug**: tracked
  agent IDs churn almost every frame even with only 1-2 real agents nearby
  (25 IDs minted in ~12s for a 2-agent scenario) - sensor position noise
  intermittently fails `sensorFusion`'s/`objectTracking`'s nearest-neighbor
  gating for the same physical object. Silently benign for safety
  (collision checking works on positions, not IDs) but makes ID-based
  triggers, and any future ID-dependent logic, unreliable until fixed.
  Not fixed yet - flagged for a dedicated pass.

Path smoothness for the five validated runs: `cattleCrossing` 0.51 rad
(smoothest - the clean early yield-then-pass), `urbanIntersection` 1.38 rad,
`marketArea` 1.99 rad, `villageRoad` 2.06 rad, `highwayMerge` 8.02 rad
(curviest - the ramp merge plus overtake genuinely requires more steering,
not a regression).

## Pipeline

```
sensors (camera/lidar/radar) -> perception -> tracking -> prediction
    -> decision -> planning -> control -> vehicle model -> [ego state feeds back to sensors]
```

See [docs/architecture.md](docs/architecture.md) for full function signatures and
the closed-loop data-flow diagram.

## Folder guide

| Folder | Purpose |
|---|---|
| `config/` | Struct schemas / config builders: simulation, vehicle, planner, agent, ego state |
| `perception/` | Per-sensor detection stubs + fusion + tracking |
| `prediction/` | Motion models that project tracked agents forward in time |
| `planning/` | Global route, local candidate generation, adaptive selection, collision check, smoothing |
| `decision/` | Behavior-level state machine (yield, overtake, stop, crawl, etc.) |
| `control/` | Trajectory-tracking controllers and the vehicle (bicycle) model |
| `scenarios/` | The 5 target Indian-road scenarios (village, intersection, highway merge, market, cattle crossing) |
| `evaluation/` | Metrics: TTC, path smoothness, replanning latency, completion rate |
| `visualization/` | Plotting helpers for objects, trajectories, paths, demo figure |
| `tests/` | Unit test stubs for collision check, planner, prediction |
| `results/` | Generated `figures/`, `tables/`, `logs/` (gitkept, populated at runtime) |
| `docs/` | Architecture, methodology, final report |
| `demo/` | End-to-end demo runner |

## Build budget

30-hour hackathon build. This phase (Hour 0-1) only sets up structure and
interfaces so Phase 1 onward is pure implementation, not design churn.
