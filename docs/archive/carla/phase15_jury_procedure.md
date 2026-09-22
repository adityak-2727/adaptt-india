# Phase 15 jury procedure

**Read the latest validation report first. A failed turn/contact run is not a
qualified jury demonstration. Do not describe this as zero-collision autonomy.**

1. Close any earlier demo using its normal cleanup. Start the installed CARLA
   0.9.16 server. Do not run another CARLA controller concurrently.
2. Open MATLAB R2026a in `C:\Users\ADITYA\sih-autonomous-india`.
3. Run `addpath(genpath(pwd)); out = runCarlaIndianHeroDemo('jury',1000);`.
4. Check the console for Town03, **10/10 traffic, 5/5 parked, 2/2 pedestrians,
   8/8 defect props, 12/12 clutter, six frozen lights, zero failed spawns**.
   Required-spawn or sensor failures stop the wrapper. Read the actual error.
5. Show the overview in the new timestamped `results/phase15/` folder. Identify
   the four approaches, surrounding town, parked cars, two-wheelers and stalls.
6. Show live RGB and LiDAR. Explain that CARLA actor metadata supplies primary
   class/pose/velocity validation; this is not a trained camera detector.
7. Point to track labels and the magenta, four-second predicted trajectories.
8. Point to actual decision, TTC (including legitimate `Inf`), centre proximity,
   and the geometric collision flag. Centre proximity is not body clearance.
9. Show candidate paths, the selected green path and the blue measured drive.
10. Let the closed loop approach, steer through the turn and reach the exit.
    Do not intervene with position updates or CARLA autopilot.
11. Read final goal/heading/path/timing and physical collision counts. If contact
    occurs or the run aborts, show it and explain the limitation.
12. The wrapper brakes, preserves results and destroys its own actors/sensors.
    Confirm `orphanActorIds` is empty and `cleanupException` is empty in
    `summary.json`. A null/unavailable measurement is not a zero.

The script uses the existing 12 m reference turn and local/adaptive planners;
the reference route is a road-route input, not a replayed ego trajectory.
Town03's baked markings remain visible. Debris/cone assets represent road
defects, not deformable potholes. The Vespa is a scooter substitute for an
auto-rickshaw representation, not an actual rickshaw asset.

Each run creates a new directory. Never rename a failed run into a successful
one or overwrite it. `run.mat` contains all retained reasoning outputs,
traffic telemetry, timing and collision events; PNGs are actual captured
frames/plots. Contact-event spans group events from the same actor separated
by at most 0.5 s; they estimate event persistence, not continuous contact.

The MATLAB wrapper is the new evaluator entry. The existing Simulink model
hosts the same `carlaClosedLoopStep` but retains its legacy traffic script;
do not claim it includes the new wrapper/dashboard. Simulink validation must
be described separately using its own measured results.

## 30-second explanation

“This is our high-fidelity urban intersection demonstration for SIH 26037.
The traffic lights are visible but do not assign right-of-way. Live CARLA
observations enter MATLAB fusion, tracking, prediction and risk evaluation.
Our planners select a path, and the controller sends steering, throttle and
brake to CARLA physics. The display shows those actual outputs. Physical
collision events are reported separately; contact-recovery robustness remains
an open limitation.”

## One-minute explanation

“Our broader MATLAB/Simulink framework covers five Indian road scenarios.
CARLA concentrates on one urban four-way junction with mixed traffic,
pedestrians, parked vehicles and road-defect representations. It is
operationally unsignalized: the autonomy stack has no traffic-light input.
CARLA provides actor metadata for class, pose and velocity validation, while
RGB, LiDAR and radar streams are acquired through the live sensor bridge.
You can see fused tracks, behavior labels, predicted motion, TTC and candidate
paths. The selected path feeds the existing controller, which physically
drives the ego through steering, throttle and brake commands. No autopilot or
ego trajectory replay is used. We report measured asynchronous timing,
physical contacts and cleanup results, including failed runs.”

## Three-minute technical explanation

“SIH 26037 concerns adaptive path planning and collision avoidance on
unstructured Indian roads. Our synthetic MATLAB/Simulink scenario framework
retains village, urban, highway-merge, market and cattle-crossing cases. The
CARLA demonstration concentrates visual and integration effort on one town
intersection, rather than presenting several shallow simulator scenes.

“The scene uses Town03 with fixed spawn coordinates and heterogeneous actor
intentions. NPCs have different speeds, start times, crossing and turning
intentions. Their motion uses vehicle physics, and walkers use WalkerControl.
Signals are frozen infrastructure. No traffic-light state reaches the ego's
decision or planning inputs. Stock lane paint remains visible, but this
pipeline does not use a lane-marking detector. The available generic assets
approximate Indian roadside clutter; we explicitly identify substitutions.

“The feedback loop begins with CARLA observations. RGB, LiDAR and radar are
acquired and timestamped; simulator actor metadata is used for primary
classification and pose validation. We do not present that as deep-learning
camera detection. The existing fusion output is converted into the common
project world frame, tracked, behavior-classified and passed to trajectory
prediction. The dashboard shows those actual tracks and predicted paths.

“Decision logic uses observed and predicted traffic. Local planning creates
candidate paths, the adaptive planner selects among them, collision checking
reports risk and TTC, and path smoothing supplies the controller. The curved
reference route defines the intended exit, but the ego's actual movement is
produced only by controller commands and CARLA physics. The blue driven path
and measured heading let us distinguish an executed turn from merely drawing
a curved path. The Simulink model also hosts this same full closed-loop step;
its validation results are reported separately.

“We distinguish predicted geometric collision flags from physical collision
sensor events. Infinite TTC is displayed as infinite, not replaced by a
plausible-looking finite number. Timing is measured on the asynchronous loop;
we do not claim zero latency or guaranteed real-time execution. Run artifacts
include failed attempts, collision actor identities and cleanup verification.
An intermittent physical-contact deadlock remains a known limitation. If it
occurs here, we stop and report it rather than moving the ego artificially or
changing the frozen planner to make the presentation appear successful.”
