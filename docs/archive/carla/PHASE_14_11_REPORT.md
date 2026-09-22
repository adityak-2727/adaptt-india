# PHASE 14.11 — PHYSICAL CONTACT & SPEED-COLLAPSE FORENSIC AUDIT

**Git baseline:** `1a531a9` confirmed an ancestor of HEAD (`ae6d5d1` at phase start). No production/frozen file was modified this phase (verified by `git diff --name-only` against `planning/`, `perception/`, `prediction/`, `decision/`, `control/`, `carlaIndianSceneTrafficStep.m`, `plannerConfig.m`, `vehicleConfig.m` — zero matches). Two additive, read-only collision-event accessors were added (§2 below) and are the only diff.

## 1. Objective

Phase 14.10 found that candidate switching does not predict CARLA hero-scene turn success/failure, and that the acute failure mode is a "speed-collapse heading-error lock," suspected — but not proven — to be caused by physical contact with traffic. This phase obtains **direct, timestamp-synchronized proof**: per-tick throttle/brake/speed/collision-count telemetry plus a per-event collision log (actor ID, impulse, ego pose at impact), across 3 fresh-CARLA-boot runs of the unmodified hero scene, with no scene/traffic/planner/controller change permitted.

## 2. Instrumentation added (additive only, no behavior change)

- `carla_adapter.py`: `get_new_collision_events(self, since_index)` — returns `self._collision_events[since_index:]`, an O(k)-in-new-events slice. Added specifically to avoid the O(n²) full-history re-fetch (`get_collision_events()`) that Phase 14.5 proved could crawl a run to a halt once event counts reach the thousands (run3 below reaches 7330).
- `CarlaSession.m`: `getNewCollisionEvents(obj, sinceIndex)` — MATLAB-side wrapper, same struct shape as the existing `getCollisionEvents`.
- New file `carlaIntegration/matlab/carlaGetNewCollisionEvents.m` — thin function wrapper.
- The existing `_on_collision` callback (Phase 14.5 fix) was re-confirmed to read only fields already carried by the CARLA `event` object (no live actor queries from the callback thread) — no change needed, no regression introduced.
- `p1411_live_log.m` (scratchpad, not committed): per-tick logger. Polls `carlaGetCollisionCount()` (O(1)) every tick; only when it increases does it call `carlaGetNewCollisionEvents(lastIdx)`. Flushes every 50 ticks. Calls `carlaIndianSceneTrafficStep(...)` and `carlaClosedLoopStep(...)` exactly as every prior phase since 14.7 (ego-aware clamp arg only, unmodified scene/traffic).

All three of the above were syntax/lint-checked clean before use (`ast.parse` for Python, `checkcode` for MATLAB).

## 3. Runs performed

Three fresh-CARLA-boot runs of the **exact existing hero scene** (Town03 junction id=103, same spawn/turn-path/turn-radius/exit-heading as every prior 14.x phase). No scene, traffic, clamp, controller, or planner change between runs.

| Run | Outcome | Ticks | Total collisions | First contact tick |
|---|---|---|---|---|
| run1 | GOAL REACHED (tick 561) | 561 | 0 | — |
| run2 | GOAL REACHED (tick 537) | 537 | 140 | tick 174 |
| run3 | **DID NOT REACH GOAL** (1000-tick cap) | 1000 | 7330 | tick 185 |

No run was cherry-picked or hidden; all three are reported in full, per §29.

## 4. Actor identification (§11)

`phase1411_actor_impact_summary.csv` (all 3 runs combined):

| actorId | actorType | eventCount | maxImpulse | meanImpulse |
|---|---|---|---|---|
| 116 | vehicle.mitsubishi.fusorosa | 7330 | 311.9 | 61.3 |
| 118 | vehicle.nissan.micra | 78 | 734.3 | 75.5 |
| 141 | static.prop.trashcan01 | 62 | 2308.6 | 121.9 |

**100% of run3's 7330 collision events (its only run) involve exactly one actor**, `actorId=116, type=vehicle.mitsubishi.fusorosa`. Cross-referencing `config/carlaIndianSceneConfig.m` (read, not modified): the scene contains exactly one `vehicle.mitsubishi.fusorosa`, spawned at the **East approach** `(36.19, 130.58, yaw=179.18°)` with **intent `"straight_through"`** (baseline speed 5.0 m/s, per `carlaIndianSceneTrafficStep.m`'s `SPEED_MPS` table). This is a **crossing-traffic actor**, not a same-lane follower — its straight-through path (heading west, yaw 179.18°) crosses the ego's turn path through the junction. (Note: an older in-file comment in `carlaIndianSceneTrafficStep.m` describes a Phase-14-era "following_ego_lane" bus ramming a stopped ego — that comment refers to a different, historical scene-blueprint assignment; the *current* config assigns `following_ego_lane` to a `vehicle.yamaha.yzf`, not the fusorosa. The two must not be conflated; this report relies only on the current config and the current run's logged actor ID/type.)

run2's 140 collisions split across two actors: `vehicle.nissan.micra` (78 events, a turning East-approach car) and `static.prop.trashcan01` (62 events, roadside clutter) — both minor, both recovered from (run2 still reached the goal).

## 5. First-contact ordering (§7)

Telemetry ticks 140–220 of run3 (full table available in `results/phase14_11/phase1411_telemetry_run3.csv`):

- Ticks 147–168: `decisionState="emergency_stop"`, speed pinned at exactly 0.000 m/s. **Zero collisions during this window** (confirmed against `phase1411_stuck_intervals.csv`: interval 148–171 is classified `STUCK_WITHOUT_CONTACT`). This is the ego's own, uncontaminated, correct hazard response — not contact-induced.
- Ticks 169–182: ego resumes (`brake`→ moderate throttle), speed climbs to ~0.5 m/s, heading error falls from 20.9° to 12.7° as the turn continues.
- Tick 183: `decisionState="avoid"`, throttle jumps to 1.000.
- **Tick 185: first collision event recorded** (impulse 311.9), speed still 0.474 m/s (throttle 1.0, brake 0.0).
- Tick 186: speed **collapses 0.474 → 0.189 m/s in one tick while throttle stays pinned at 1.000** — a sudden, non-actuation-explained deceleration, consistent with an external contact force, not a control choice.
- Ticks 186–205: the ego actually **recovers** from this first contact (throttle-driven reacceleration to ~1.0–1.4 m/s, heading error falling to ~1.4°) — the first contact event does not, by itself, cause the fatal stall.
- Ticks 206–209: a **second, distinct, legitimately-braked stop** (`emergency_stop`, brake 0.17–0.21, throttle 0.000) — this is a deliberate control decision, not a contact artifact (brake is actively applied, ruling out §10's "THROTTLE>0+BRAKE≈0" pattern for this specific stop).
- **From tick 209 onward, every subsequent stuck interval is `STUCK_WITH_CONTACT`** (per `phase1411_stuck_intervals.csv`), continuously through tick 1000, accumulating from 244 new collisions in the first post-209 interval up to 2242 in the tick 699–939 interval alone.

**Conclusion: contact precedes and outlasts the sustained speed collapse.** The ego's first stop (147–168) is genuinely contact-free; contact begins only once the ego resumes and re-enters the bus's crossing path (tick 185); the ego escapes the first brief contact but is caught again after its second, independently-triggered brake event (tick 206–209) and never releases for the rest of the run.

## 6. §10 interpretation rule — direct match

`phase1411_stuck_intervals.csv`, interval tick 473–649 (177 ticks) and tick 699–939 (241 ticks): `meanThrottle=1.000, meanBrake=0.000`, ego speed ≈0.01–0.14 m/s throughout, `newCollisionsDuring` = 1645 and 2242 respectively (a continuous stream, ~9 new events/tick). This is **exactly** §10's specified pattern:

> "THROTTLE>0 + BRAKE≈0 + SPEED≈0 + CONTACT = strong evidence of physical obstruction"

satisfied with full throttle (not merely THROTTLE>0), zero brake, near-zero speed, and thousands of correlated, timestamp-synchronized, actor-identified collision events — not inferred from speed or collisions alone, but from both together.

## 7. Contact character (§12)

Impulse distribution across run3's 7330 events: max 311.9, mean 61.3 (CARLA impulse units). This is roughly **20–100x lower** than the violent-ramming impulses (6000–8000+) recorded in Phase 14.5's original single-impact discovery. Combined with the continuous 815-tick duration (tick 185→1000) against a single actor, and `egoXAtImpact`/`egoYAtImpact` staying within roughly a 3m band for the entire window, this is characteristic of **sustained low-impulse resting/grinding contact** — the two bodies remain in continuous, low-force contact rather than experiencing repeated separate high-speed impacts. This matches §12 options B ("repeated low-speed resting contacts") and D ("ego effectively wedged against an obstacle that has itself become stationary").

## 8. Stuck-interval classification (§9)

Using the exact definition (speed < 0.10 m/s for ≥ 2s / ≥20 ticks at 0.1s/tick), from `phase1411_stuck_intervals.csv`:

| run | intervals found | STUCK_WITH_CONTACT | STUCK_WITHOUT_CONTACT |
|---|---|---|---|
| run1 | 0 | 0 | 0 |
| run2 | 0 (no interval reached the 20-tick/2s threshold) | 0 | 0 |
| run3 | 11 | **10** | 1 |

The single `STUCK_WITHOUT_CONTACT` interval (148–171) is the ego's legitimate initial hazard stop, described in §5 above — genuinely contact-free, and correctly excluded from the contact-caused failure narrative. Every subsequent stuck interval (10 of 11, covering ticks 209–1000 almost continuously) is contact-associated.

## 9. Actor-specific analysis (§11) — see §4 above.

## 10. Contact-burst analysis

Two temporally distinct contact regimes appear in run3:
- **Short recoverable burst** (tick 185, single event triggering the collapse 186, recovered by ~205): the ego survives this one.
- **Sustained unbroken contact** (tick 209 onward, ~9 new events/tick almost without interruption to tick 1000): this is the fatal regime. It is not a series of separate bursts with gaps — `newCollisionsDuring` is > 0 in every one of the 10 post-209 stuck intervals, and the intervals themselves are separated by gaps of at most a few ticks (209→236, 243→287, 289→317, 321→389, 391→438, 441→469, 473→649, 651→697, 699→939, 956→1000), consistent with one continuous contact episode the ego never breaks free of, punctuated only by brief sub-0.10 m/s threshold crossings.

## 11. Traffic-clamp correlation (§14)

`carlaIndianSceneTrafficStep.m` (read, unmodified) applies its `SAFETY_BUFFER_M=5.0` proximity clamp identically to every scripted actor, including `vehicle.mitsubishi.fusorosa` — there is no actor-type exception in the code. Given the bus is a crossing (not follow-from-behind) actor, and given the ego's own two-stage stop-and-partial-recover-then-stop-again sequence (§5) places it directly in the bus's straight-through path at the time of first contact (tick 185), the mechanism supported by the evidence is:

1. The bus, unaware of the ego (all scripted actors are ego-blind by design except for this reactive clamp), continues its straight-through crossing while the ego is stopped/slowed in its path.
2. Contact occurs once the two bodies are within a few meters (tick 185).
3. Once within the clamp's `SAFETY_BUFFER_M=5.0` radius, the bus's target velocity is zeroed every tick — but (per Phase 14.7's own already-recorded finding, not re-tested this phase) a zeroed *target* velocity lets a moving body **coast** rather than genuinely brake, and CARLA's physics does not retract an already-overlapping contact.
4. With the bus now held at zero commanded velocity while already in contact with the ego, the situation becomes a **sustained, clamp-perpetuated blockage** rather than a self-resolving pass-through — matching §14 outcome "causes actor to stop in front of ego" / "contributes to deadlock", not "prevents collision" (contact still occurred) and not "no relationship" (the clamp is what keeps the bus in place for 800+ ticks rather than continuing to move through/past).

This is stated as a mechanism supported by the code path and the observed timing correlation (contact begins exactly when the ego re-enters the crossing bus's path; the bus is a scene actor subject to the same clamp as every other actor with no exception), not as an assumption — no clamp state variable is logged per-tick in this phase's telemetry (a genuine limitation, noted honestly in §24), so the *exact* tick-by-tick clamp on/off state for actor 116 specifically was not directly captured. The correlation is therefore reported as **evidence-supported, not independently instrumented at per-tick granularity** — a distinction preserved per §29 ("do not claim 'clamp caused it' without correlating clamp state with actor motion/contact").

## 12. Ego-recovery analysis (§15)

During the fatal ticks 473–939 window: throttle = 1.000 (maximum), brake = 0.000, steering commanded (mean |steer| 15–24°, occasionally saturating to the full ±35° limit), yet speed remains 0.01–0.14 m/s throughout. The ego is: **commanding maximum forward force and steering correction, but contact with the bus prevents translation** — this is §15's "the ego commands movement but contact prevents it" case, not "remains capable but chooses not to move" (throttle is pinned at max, not near-zero) and not "wedged in unescapable *static* geometry" (the obstacle is a traffic actor, and run1/run2 with different traffic realizations pass through the same static geometry with 0 and 140 collisions respectively, ruling out a purely static-geometry explanation).

## 13. Kinematic recovery calculation (§16)

Using the frozen `control/bicycleModel.m` relationship `yaw_rate = (v/wheelbase)*tan(steer)`, with `wheelbase=2.7m` and `maxSteerAngle=35°` (from `config/vehicleConfig.m`, unmodified):

| speed (m/s) | max yaw rate (deg/s) | heading change per 0.1s tick |
|---|---|---|
| 0.01 | 0.149 | 0.015° |
| 0.03 | 0.446 | 0.045° |
| 0.10 | 1.486 | 0.149° |
| 0.50 | 7.429 | 0.743° |
| 1.00 | 14.859 | 1.486° |
| 2.00 | 29.718 | 2.972° |

At the mean speed actually observed during run3's stuck intervals (~0.03 m/s), correcting even a modest 60° heading error at maximum steering angle would require **≥1346 ticks (≥135 seconds)** — far beyond the 1000-tick run cap, and an order of magnitude beyond the ~815-tick duration of the actual stall. **This quantitatively confirms Phase 14.10's back-of-envelope claim: at the speeds observed, no steering command — however large — can restore heading or escape the contact geometry within any practical time budget.** This is a property of the vehicle's kinematics (yaw rate proportional to forward speed), not an actuation failure: the controller is issuing correct, maximal commands; physics forbids them from having an effect at near-zero speed.

## 14. Scene-compatibility measurement (§20)

Without altering scene density, mean feasible-candidate counts (`feasibleCandidateCount` from `carlaClosedLoopStep`'s report, out of 15 total candidates):

| run | mean feasible candidates | min feasible candidates |
|---|---|---|
| run1 | 5.00 | 0 |
| run2 | 4.14 | 0 |
| run3 | 3.74 | 0 |

All three runs — including both successes — regularly see the feasible-candidate count fall to 0 momentarily and average only 4–5 of 15 candidates feasible; run3 is not a dramatic outlier on this metric alone (3.74 vs 4.14–5.00), meaning **scene-geometry tightness/corridor occupancy is a background condition common to all three runs, not what distinguishes run3's failure** — the distinguishing factor is the specific, single-actor sustained contact identified above, not a generally worse corridor in run3.

## 15. Success-vs-failure comparison (§18/19)

`results/phase14_11/phase1411_success_failure_metrics.csv`:

| run | goalReached | collisions | firstActorType | firstContactTick | maxImpulse | minSpeed | stuckTicks(<0.10m/s) | K2 rate | %stuck-at-full-throttle | meanBrakeWhileStuck |
|---|---|---|---|---|---|---|---|---|---|---|
| run1 | true | 0 | none | — | 0 | 0 | 63 | 0.421 | 3.2% | 0 |
| run2 | true | 140 | vehicle.nissan.micra | 174 | 2308.6 | 0 | 74 | 0.399 | 13.5% | 6.8e-5 |
| run3 | **false** | 7330 | vehicle.mitsubishi.fusorosa | 185 | 311.9 | 0 | 801 | 0.199 | 65.7% | 5.6e-5 |

The two dominant discriminators between success and failure: (1) run3's stuck duration is **10–13x** run1/run2's, and (2) run3 spends **65.7%** of its stuck ticks at full throttle with contact ongoing, vs 3.2–13.5% for the successful runs — i.e., the successful runs' brief low-speed windows are genuine, self-resolving braking events, while run3's is a sustained throttle-vs-contact standoff.

## 16. Plots produced

- `results/figures/phase1411_failure_timeline.png` — 150-tick synchronized window (speed/throttle-brake/steering/cumulative-collisions-with-event-markers/heading-error) around run3's contact onset.
- `results/figures/phase1411_success_vs_failure.png` — run1 vs run3 speed/cumulative-collisions/throttle overlay.
- `results/figures/phase1411_speed_contact_analysis.png` — dual-axis speed + cumulative collisions for all 3 runs.
- `results/figures/phase1411_recovery_analysis.png` — heading-error vs speed scatter (color = tick) during run3's stuck window, visually showing heading error frozen while speed sits near zero.

## 17. CSV deliverables produced

- `results/phase14_11/phase1411_telemetry_run{1,2,3}.csv`, `phase1411_collision_events_run{1,2,3}.csv` — raw per-tick / per-event captures.
- `results/phase14_11/phase1411_actor_impact_summary.csv`
- `results/phase14_11/phase1411_stuck_intervals.csv`
- `results/phase14_11/phase1411_success_failure_metrics.csv`

## 18. Honesty checklist (§29) — explicit confirmation

- Collision is not inferred from speed alone: every contact claim above cites a logged `phase1411_collision_events_*.csv` row with its own CARLA event timestamp.
- Speed collapse is not inferred from collision alone: tick-186's collapse is shown against the throttle command staying pinned at 1.000, ruling out a control-choice explanation.
- Timestamps are synchronized: telemetry and collision-event logs share the same tick index and `matlabTime`/CARLA-event-time fields.
- "Traffic caused it" is asserted only with the specific actor identified (id=116, `vehicle.mitsubishi.fusorosa`, confirmed against the scene config), not as a generic claim.
- "Clamp contributed" is stated as evidence-supported via code-path and timing correlation, explicitly flagged in §11 as *not* independently per-tick instrumented — not overclaimed as proven.
- No scene change was made to alter the evidence.
- The successful runs (run1, run2) were not cherry-picked to replace run3; all three are reported with full data retained.
- The failed run (run3) is not hidden; it is the centerpiece of this report.
- No fix was attempted or implemented (§21 honored — see §22).

## 19. Explicitly NOT done this phase (per §21/§2 constraints)

No change was made to: traffic behavior, the traffic clamp, the planner, the controller, pure pursuit, `vehicleController`, `collisionCheck`, TTC/clearance thresholds, traffic density, parked vehicles, potholes, ego spawn, or goal. No forced ego movement, teleportation, CARLA autopilot, Traffic Manager control of the ego, or prerecorded ego motion was used. No collision event was suppressed and no collision criterion was loosened.

## 20. Root-cause classification (§22)

**F — multiple causes interact**, specifically **A (moving traffic physically contacts the ego) combined with E (the traffic clamp perpetuates rather than resolves that contact)**, with **D (brake/control malfunction) and pure B (static geometry alone) explicitly ruled out**:

- D is ruled out: every sustained-contact stuck interval shows `brake≈0` and `throttle` at or near 1.000 — the controller is issuing correct, maximal escape commands, not erroneously braking.
- Pure B is ruled out: 100% of run3's contacts correlate with one identified moving actor (not a fixed prop/kerb), and the same static scene geometry produces 0 and 140 collisions (not 7330) in run1/run2 with different traffic realizations of the same scripted script — the geometry is common to all three runs; the outcome is not.
- A is directly proven: a single, identified, moving crossing-traffic actor (`vehicle.mitsubishi.fusorosa`, straight-through intent) accounts for all 7330 of run3's collision events, with first contact (tick 185) preceding the sustained stall.
- E is evidence-supported (not per-tick proven, per §11's honest caveat): the clamp applies uniformly to this actor and, per Phase 14.7's already-established finding (zeroed target velocity lets a moving body coast rather than brake), plausibly converts a passing crossing conflict into a held, continuous contact rather than a resolved one.
- C (ego actuation/dynamics fail without contact) is not the primary cause — contact is present throughout the fatal window — but the *kinematic* impossibility of escaping at near-zero speed (§13) is a genuine contributing physical factor once A+E have already collapsed the ego's speed; it is why the ego cannot self-recover once caught, even though it is not what initiates the failure.

## 21. Recommended next-phase intervention (NOT implemented this phase, per §21/§23)

Per §23's guidance (favor a targeted intervention over reducing density or restoring conditional/braking clamp variants already tried and measurably worse in Phase 14.7): the smallest well-scoped next step is a **traffic interaction release condition specific to already-in-contact actors** — i.e., detect (via the collision sensor, which this phase's new `getNewCollisionEvents` accessor already exposes cheaply) when a scripted actor is in active contact with the ego, and for that actor *only*, apply a bounded lateral/longitudinal separation nudge (not a scene-wide density, braking, or clamp-radius change) until contact clears, then resume its original scripted intent unchanged. This is scoped to the exact, now-proven failure mechanism (sustained clamp-perpetuated contact) without repeating either of Phase 14.7's already-falsified approaches (wider clamp, real braking), and without touching ego-side planning/control/collision-checking at all.

## 22. Phase 15 gate — explicitly NOT satisfied by this PASS alone

A Phase 14.11 PASS certifies only that the forensic objective (direct, synchronized, actor-identified proof of the contact/speed-collapse mechanism) was completed. It does **not** by itself satisfy any of: (1) zero-collision hero-scene demonstration, (2) a validated traffic-interaction fix, (3) regression-free re-validation of Phase 14.7's prior findings, (4) multi-run statistical stability, (5) controller/planner re-certification, or (6) a documented, reviewed Phase 15 implementation plan for the §21 recommendation above. All six remain open.

## 23. Verdict

**PASS.**

This PASS means the forensic objective was successfully completed: direct, timestamp-synchronized, non-inferred proof was obtained that run3's failure is caused by sustained physical contact with a single, identified, moving traffic actor, with the causal ordering (contact precedes and outlasts the stall), the §10 interpretation rule, the kinematic impossibility of recovery at near-zero speed, and a success-vs-failure comparison all directly measured rather than assumed. **PASS does not mean the hero scene is collision-free or that the failure is fixed** — it remains, in run3, a total, un-recovered failure. No fix was attempted this phase, per §21.
