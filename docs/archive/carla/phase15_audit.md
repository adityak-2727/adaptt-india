# Phase 15 initial audit — 8 September 2026

Baseline: main, f226210f4ccccbbab31d35c5fafd509b2b357f9c.
Working tree: only untracked results/phase14_12/; preserve all its contents.
No AGENTS.md was found in the project. No commit is authorized.

These are implementation-inspection statuses, not fresh runtime passes.

| Requirement group | Initial status | Evidence / gap |
|---|---|---|
| One large Town03 four-way urban scene | PARTIAL | Existing config documents junction 103, four approaches; fresh visual verification needed |
| Indian character, parked cars, defects | PARTIAL | Five parked vehicles, eight defect props, roadside stalls/clutter; stock town, baked markings and generic debris remain limitations |
| Mixed traffic / pedestrians | PARTIAL | Ten vehicles, two walkers; spawn and movement need fresh verification |
| Non-controlling lights / no new markings | PASS | Builder freezes lights; no signal input in closed-loop step |
| Independent plausible traffic intentions | PARTIAL | Deterministic intentions exist, but instantaneous velocity rotations are not plausible steered turns |
| Genuine ego turn / architecture | PARTIAL | Existing curved route feeds frozen local/adaptive planning and controller, then VehicleControl; needs fresh completion evidence |
| Deterministic configuration | PASS | Fixed spawn manifest, no random primary traffic |
| Repeatable physical outcome | MISSING | Async timing and known contact deadlock preclude assuming reproducibility |
| Clean evaluator entry point | MISSING | Phase 14 entry runs six demos, lacks strict required-spawn checks |
| Sensor reliability / fail-safe | PARTIAL | Four sensors available; missing observations can currently skip control without stopping |
| Cleanup verification | MISSING | Adapter destroys owned actors but catches some destroy failures without verifying zero survivors |
| Live reasoning dashboard | MISSING | Existing Phase 14 evidence is largely final static figures |
| Honest sensor provenance | PASS | Adapter documents simulator actor metadata, not a trained RGB detector |
| Fresh performance / evidence / three starts | MISSING | Must measure all new attempts, including failures |
| Simulink | PARTIAL | Existing closed-loop System block/model; fresh validation needed |
| Frozen files / regressions | PARTIAL | Clean tracked baseline recorded; final diff and suites required |

Scope: reuse the scene and complete closed-loop step. Add an evaluator wrapper,
live display and auditable run artifacts. Do not tune the frozen planner,
controller, vehicle settings, collision assertions or the 5 m contact clamp.
The Phase 14 contact issue remains unresolved.
