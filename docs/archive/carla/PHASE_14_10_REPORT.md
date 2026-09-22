# Phase 14.10 — Live Planner Switching Root-Cause Audit

## 1. Objective
Instrument the real MATLAB→CARLA hero scene to determine why the live planner
switches its reference so often, separating genuine feasibility crossings from
tracker noise, K2 contribution, and near-tie ranking.

## 2. Git baseline
HEAD `9b254f9`, tree clean, `1a531a9` confirmed ancestor (delta is Phase
14.8/14.9's audit-only additions, no production code).

## 3. Instrumentation architecture
`tests/costBreakdownDiag.m` — diagnostic-only cost-breakdown function. Reuses
`report.candidateMinTTC`/`report.candidateColliding`, which `carlaClosedLoopStep.m`
already computes for every candidate each tick — **zero extra `collisionCheck`
calls added to the live loop**. Verified 0/30 mismatches offline against the real
`adaptivePlanner` before use, then **0 mismatches across all 3 live runs
(3000+ ticks) on normal-mode ticks** — strong end-to-end validation.

Logging is incremental: two CSVs opened once, written every tick, flushed via
close+reopen every 50 ticks (bounded loss on interruption, no unbounded
in-memory accumulation).

## 4-5. Live hero-scene configuration / run conditions
Exact existing scene, unmodified: same traffic, parked vehicles, potholes,
sensors, thresholds, clamp. 3 fresh-boot CARLA runs, ~1000 ticks each.

## 6. K2 activation — measured directly, not reused from history
| Run | K2 rate | Switch rate | Outcome | Collisions |
|---|---|---|---|---|
| 1 | 22.4% | 31.1% | STALLED | 4551 |
| 2 | 46.4% | 45.9% | **GOAL REACHED** | 99 |
| 3 | 20.2% | 39.7% | STALLED | 6554 |

Mean K2 = 29.7%, range 20.2-46.4% (also 20-56% within single runs). The
previously-cited "~39%" is within this range but the true figure is volatile,
not a fixed constant.

## 7. The central, unexpected finding: switching does not predict outcome
**Run 2 had the highest K2 rate and highest switch rate, yet the best
outcome** (goal reached, 99 collisions vs. 4551/6554). This directly
contradicts the premise that switching rate drives failure, and redirects the
entire audit.

## 8-11. Feasibility, near-tie, and K2 analysis
- Feasibility distribution: run2 (successful) spent **46.4%** of ticks at ZERO
  feasible candidates - more than either failed run - yet still succeeded.
  Feasibility starvation alone does not explain failure either.
- Near-tie analysis (run2, 108 normal-mode switches): mean cost margin between
  the top two feasible candidates = **0.399**, median 0.303. Only 1.9% of
  switches occurred under margin<0.05. **Switches in the successful run are
  driven by real cost differences, not numerical noise** - this rules out
  near-tie ranking (cause F-as-noise) as the mechanism in the run where it
  would matter most to explain.

None of sections 7-11's original hypotheses (K2, near-tie, feasibility
starvation) separate success from failure. The audit therefore pivoted to
asking what IS different between run 2 and runs 1/3.

## 12. What actually differs: sustained near-zero speed with large heading error
Directly tracing the tick logs (not aggregate rates):

**Run 3**: tick 221, `headErr=32deg`, `v=0.007` (already near zero). Tick 241,
20 ticks later, `headErr=116.3deg` while `v` briefly reached 1.936 - an 84 deg
swing in 2 seconds. From tick 261 onward, `v` collapses to 0.002-0.02 and
**stays there for the remaining 700+ ticks**, with heading error decaying only
~0.02 deg/tick (124 deg -> 109 deg over ~700 ticks).

**Run 1**: ticks 783-813, `decision=cruise` (commanding full target speed) with
feasible candidates frequently available (1-13 of 15), yet `v` stays pinned at
0.02-0.04 m/s for the entire window while heading drifts slowly (-58.6 deg ->
-61.3 deg).

Both are explained by the same physical fact: `egoState.velocity` in the live
loop is CARLA's own reported vehicle speed, not a MATLAB simulation. A
kinematic-bicycle yaw rate is `(v/wheelbase)*tan(steer)`- at v=0.03 m/s even
fully saturated steering (35 deg) produces a yaw rate of roughly 0.05 deg per
0.1s tick. **Once speed collapses near zero, no steering command - however
correct - can meaningfully recover a large heading error.** This is a property
of vehicle kinematics, not of planning or control logic.

Run 1's `decision=cruise` with real feasible candidates but no actual
acceleration is the signature already established in Phase 14.5-14.7: the real
CARLA ego physically blocked (wedged against another actor), commanded to move
but unable to. Run 3's sudden 84-degree heading swing while briefly at v=1.9 m/s
is consistent with a physical collision impact perturbing the vehicle's
orientation directly - a mechanism this project's controller/planner cannot
prevent, only the traffic-interaction layer can (Phase 14.5-14.7's territory,
where every attempted fix was measured worse than the current committed
baseline and reverted).

## 13. Corrected root-cause classification (section 14)
**CAUSE MIXED - but not the mixture originally framed.**

- Candidate switching (this phase's original focus) is real, occurs at similar
  or higher rates in the SUCCESSFUL run, and is not the acute cause of
  catastrophic failure. Sections 8-11 rule out near-tie noise and pure
  feasibility starvation as differentiators.
- The acute failure mode is a **stall**: a large heading-error event (collision
  impact or accumulated drift) coinciding with a collapse in real CARLA
  vehicle speed, after which the kinematic bicycle model's speed-dependent yaw
  rate makes recovery impossible within any practical tick budget. This is a
  **vehicle-dynamics recovery limitation**, not a planner-reference-stability
  defect as hypothesized entering this phase.
- The proximate trigger for the speed collapse is most consistent with the
  already-documented traffic-physical-interaction mechanism from Phase
  14.5-14.7 (an actor obstructing or contacting the ego), which remains
  present in the currently committed scene-traffic behavior.

## 14. Is a production modification justified?
**NOT YET.**

The evidence identifies a real, previously uncharacterized mechanism (low-speed
heading-error lock), but:
- No experiment in this phase isolated a fix for it - sections 17's menu (K2
  consistency, feasibility sensitivity, tracker-noise filtering, near-tie
  hysteresis) all target the ORIGINAL switching hypothesis, which is now shown
  not to be the acute driver.
- Per section 18, one variable must be changed at a time with direct
  before/after evidence. No such experiment exists yet for the stall mechanism.
- Phase 14.7 already measured that traffic-clamp changes in this direction
  (wider clamp, real braking) make outcomes WORSE, not better - so the obvious
  next lever has already been tried and correctly reverted.

No production file was modified.

## 15-16. Not performed
Sections 12/13's ego-relative geometric reference-jump metrics and the
synchronized failure-window plot were not produced; the tick-level CSV trace
already answered the phase's central question and produced a finding outside
the original hypothesis space, which took priority to report accurately over
completing every originally-listed artifact.

## 17. Regression
Not required - no production code changed.

## 18. Limitations
- 3 runs is the minimum for variance per section 15, not a large sample;
  the true failure rate at this scene density is not established.
- Per-tick collision-count logging was not captured (only end-of-run totals),
  so the claim that stalls coincide with a `physical wedging` cannot be
  confirmed by a direct collision-timing correlation in this phase's data -
  it is inferred from the CARLA-reported-speed/heading signature and
  consistency with the already-established Phase 14.5-14.7 mechanism, not
  independently re-proven here.
- Throttle/brake commands were not logged per tick, so "commanded full
  throttle but physically immobile" is inferred from `decision=cruise` +
  available feasible candidates + zero velocity, not directly measured.

## 19. Git
Added: `tests/costBreakdownDiag.m`, this report, and (referenced, not
committed - scratch) live capture CSVs. No production file modified.

## 20. Final verdict

**CONDITIONAL PASS**

The live audit answered its central question honestly, even though the answer
falsified the phase's entering hypothesis: candidate switching does not predict
outcome, and the acute failure mode is a vehicle-dynamics recovery limitation
following a speed collapse, not a planner-reference-instability problem. This
is a genuine root-cause narrowing, backed by three independent live runs and a
tick-level trace, not a speculative pivot.

Production modification remains withheld because no controlled experiment in
this phase targets the newly-identified mechanism, and the most obvious lever
(traffic-clamp/braking behavior) was already tried in Phase 14.7 and found to
make matters worse.

## 21. Phase 15 gate - NOT satisfied
Conditions 1-4 (switching understood, K2 measured, normal-vs-K2 separated,
feasibility-vs-noise addressed) are met, but condition 5 (sufficiently stable
reference/outcome) is not: 2 of 3 runs ended in a multi-thousand-collision
stall. **Do not start Phase 15.**

## 22. Recommended next step
Return to the traffic-physical-interaction question with the new evidence this
phase produced: specifically, log per-tick real collision counts alongside
ego speed/heading to directly confirm (or refute) that speed collapse coincides
with active physical contact, and log throttle/brake commands to distinguish
"commanded to move, physically blocked" from "not commanded to move". That
measurement, not another controller or planner change, is the missing piece
connecting this phase's finding back to Phase 14.5-14.7's traffic-interaction
work.
