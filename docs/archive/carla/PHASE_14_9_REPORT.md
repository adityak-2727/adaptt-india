# Phase 14.9 — Planner Reference Stability & Candidate Switching Audit

## 1. Objective
Determine why the planner's selected reference path changes so frequently
during the CARLA hero turn, and whether that instability can be removed
without compromising collision avoidance.

## 2. Git baseline
HEAD `70213ae`, tree clean. `1a531a9` is an ancestor; the delta is Phase 14.8's
audit-only additions (two test harnesses + report, **no production code**). Noted
rather than reverted, since reverting would discard the prior phase's evidence.

## 3. Files inspected
`planning/adaptivePlanner.m` (complete, 237 lines), `config/plannerConfig.m`,
`planning/localPlanner.m` interface, `planning/collisionCheck.m` interface,
`config/vehicleConfig.m`.

## 4. Planner architecture (traced)
```
previousIndex + candidateTrajectories (15, fixed left-to-right lateral order)
      |
      v
per candidate: collisionCheck -> (isColliding, minTTC)
               assessCandidateAgainstPredictions -> (minClearance, uncertaintyExposure)
               curvature, impliedSafeSpeed
      |
      v
costs(c) = riskCost + clearanceCost + deviationCost + curvatureCost
           + speedChangeCost + uncertaintyCost + consistencyCost
      |
      v
safeIdx = ~isColliding & minTTC >= ttcThresholds.critical (1.5)
      |
   +--+--------------------------------+
   | non-empty                          | empty  (K2 FALLBACK)
   v                                    v
min(costs(safeIdx))                 isFeasible = curvature <= tan(35deg)/2.7
  -> uses consistency               feasible: sortrows([minTTC, minClearance], [-1 -2])
                                    else:     max(minTTC)
                                      -> NEVER references costs()
      |                                   |
      +-----------------+-----------------+
                        v
                 selectedTrajectory -> controller reference
```

## 5. Does K2 use the consistency cost? — **PROVEN NO**
Static, line-level proof from `adaptivePlanner.m`:

- Normal branch (lines 142-145): `[~, bestLocal] = min(costs(safeIdx))`. `costs`
  includes `consistencyCost` (lines 137, 139). **Consistency applies.**
- K2 branch (lines 156-172): ranks by
  `sortrows([minTTCs(feasibleIdx), minClearances(feasibleIdx)], [-1,-2])`, or
  `max(minTTCs)` when nothing is kinematically feasible. **The `costs` array is
  never referenced anywhere in this branch.** Consistency plays zero role.

**ROOT CAUSE A is confirmed as a genuine code property.** Phase 14.6/14.7
measured K2 active on ~39 % of live ticks, so on those ticks the reference is
chosen with no consistency term at all, ordered purely by `minTTC` — a quantity
that moves every tick as tracked obstacles jitter.

## 6. Consistency-cost magnitude (static)
`consistencyCost = 0.3 * |c - previousIndex| / 15`

| Change | Value |
|---|---|
| adjacent index (delta=1) | **0.0200** |
| half-lattice (delta=7) | 0.1400 |
| maximum (delta=14) | 0.2800 |

Against the dominant term `riskCost = 5.0 / minTTC`: a minTTC move from 2.0 s to
1.5 s changes cost by **0.833**, i.e. **42x** an adjacent-index switch penalty.
`clearanceCost = 2.0/minClearance` spans 0.67-20.0; `deviationCost` spans 0-2.5.
Consistency is the weakest term in the function by one to two orders of
magnitude.

## 7. Consistency effectiveness — measured (section 12)
Offline, 200 ticks, real `localPlanner` candidates, real `collisionCheck`,
obstacles at the 1.7-2.6 m lateral offsets Phase 14.6 measured for the objects
that actually rejected candidates live.

Diagnostic copy validated first: **40/40 identical selections** vs the real
`adaptivePlanner` at default settings.

| Variant | switch % |
|---|---|
| production (w=0.3, K2 without consistency) | **56.3** |
| consistency OFF (w=0) | **56.3** |
| w=1.5 (5x) | 55.8 |
| w=3.0 (10x) | 55.8 |
| w=0.3 + consistency in K2 | 56.3 |
| w=3.0 + consistency in K2 | 55.8 |

**Removing the consistency term entirely changes switching by 0.0 percentage
points. Amplifying it tenfold recovers 0.5 points.**

This confirms **ROOT CAUSE B** and, importantly, establishes a **negative
result**: raising the consistency weight is *not* the fix. That closes off the
most obvious candidate remedy before anyone spends a phase on it.

In this scenario **K2 activated on 0 % of ticks**, so the 56 % switching arises
entirely within *normal* ranking. ROOT CAUSE A is real in code but is **not**
the dominant driver of switching by itself.

## 8. Root cause
**ROOT CAUSE B (dominant) + ROOT CAUSE A (real, secondary) + ROOT CAUSE F
(implicated).**

Switching is driven by the large, fast-moving `riskCost`/`clearanceCost` terms
responding to obstacle motion and tracker jitter. Those terms swing by whole
units tick-to-tick, while the only stabilising term is capped at 0.28 and is
absent altogether on the ~39 % of live ticks that take K2. The consistency
mechanism is present in the design but numerically irrelevant.

## 9. Is a production modification justified?
**No — not on this evidence.**

Per section 20, the audit did not identify a change that is both proven and
minimal:
- Strengthening consistency is **measured not to work** (0.5 points at 10x).
- Adding consistency to K2 is defensible on principle, but K2 never activated in
  the offline scenario, so this audit produced **no measurement** of its effect.
  Changing safety-ordering code on an untested hypothesis is exactly what
  sections 20 and 29 forbid.

No production file was modified.

## 10-11. Not performed
Live CARLA candidate-cost logging (sections 6/7), the full geometric switching
decomposition (section 8), K2-vs-normal live split (section 10), candidate
identity/order validation (section 16), feasibility-boundary analysis (section
17), and input-noise sensitivity (section 18) were not completed.

## 12. Limitations — disclosed
- **The geometric displacement metric in the harness is contaminated** and is
  therefore not reported as a finding. It measures mean pointwise distance
  between consecutively selected paths, but the harness also advances the ego
  along the reference each tick (and wraps its index), so ego progress and the
  wrap discontinuity dominate the number. The `switch %` comparison across
  variants remains valid because all variants share one scenario and differ only
  in the consistency knob.
- The offline scenario is synthetic. It reproduces the *switching regime*
  (56 % vs 24-49 % live) but not the live K2 rate (0 % vs 39 %), so conclusions
  about K2's live contribution rest on static code proof, not measurement.
- No CARLA validation was run this phase (none was warranted, since no
  production change was made).

## 13. Regression
Not required — no production code changed.

## 14. Git
Added: `tests/testPhase149PlannerStability.m`, `tests/adaptivePlannerDiag.m`
(diagnostic copy, `tests/` only, never called by the pipeline), this report.

## 15. Final verdict

**CONDITIONAL PASS**

Root cause is strongly identified: the consistency mechanism is numerically
irrelevant (measured), and K2 omits it entirely (proven from code). A key
negative result — that strengthening the weight does not help — is established,
which removes the most likely wrong turn for the next phase.

Production modification is deliberately withheld as unjustified by current
evidence, and live CARLA instrumentation remains incomplete.

## 16. Phase 15 gate — NOT satisfied
Condition B is unmet: the planner neither produces a stable reference nor has a
documented, justified limitation with a remedy. **Do not start Phase 15.**

## 17. Recommended next step
Instrument the **live** planner (section 6's candidate log) on the real hero
scene to capture, per tick: each candidate's cost breakdown, feasibility, minTTC,
minClearance, and the normal-vs-K2 split. The offline harness has taken this as
far as synthetic obstacles can. The specific open question is whether the live
56 %-equivalent switching is driven by genuine feasibility-boundary crossings
(ROOT CAUSE C — obstacles really do move in and out of candidate corridors) or by
tracker noise perturbing minTTC among near-tied candidates (ROOT CAUSE F). Those
have different fixes, and the current evidence does not separate them.
