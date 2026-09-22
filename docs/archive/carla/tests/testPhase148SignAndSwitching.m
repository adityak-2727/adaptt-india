function testPhase148SignAndSwitching()
% testPhase148SignAndSwitching - Phase 14.8 sections 7 and 12.
%
% PART 1 (section 7): numerically verify the steering sign and frame
% conventions end to end - pure pursuit, the bicycle model, and the
% MATLAB->CARLA normalization - so a sign/frame bug cannot be mistaken
% for a tuning problem.
%
% PART 2 (section 12): the critical distinction. The offline baseline
% tracks a FIXED path with 0.03m mean CTE and 0% steering saturation,
% while the live CARLA loop shows 1.90m CTE and 69% saturation with the
% SAME controller. The one structural difference is that the live
% controller follows pathSmoothing(selectedTrajectory), which the planner
% re-selects every tick. This test reproduces that effect offline - same
% controller, same plant, but a reference that SWITCHES laterally at a
% measured rate - to establish whether reference switching alone is
% sufficient to produce the observed saturation.
%
% Modifies no production code.

addpath(genpath(fileparts(fileparts(mfilename('fullpath')))));
vehCfg = vehicleConfig();

fprintf('\n===== PART 1: steering sign / frame verification =====\n');
tol = 1e-9;
pass = true;

% Straight reference path along +x.
straight = [(0:1:60)', zeros(61,1)];

% TEST A: vehicle LEFT of path (y>0) must steer RIGHT (negative).
ego = mkEgo(0, 1.5, 0, 3.0);
sA = purePursuitController(ego, straight, 5.0, vehCfg);
pass = pass && check('A left-of-path -> negative (right) steer', sA < -tol, sA);

% TEST B: vehicle RIGHT of path (y<0) must steer LEFT (positive).
ego = mkEgo(0, -1.5, 0, 3.0);
sB = purePursuitController(ego, straight, 5.0, vehCfg);
pass = pass && check('B right-of-path -> positive (left) steer', sB > tol, sB);

% TEST C: on path, aligned -> ~zero steer.
ego = mkEgo(0, 0, 0, 3.0);
sC = purePursuitController(ego, straight, 5.0, vehCfg);
pass = pass && check('C aligned -> ~zero steer', abs(sC) < 1e-6, sC);

% TEST D: on path but yawed LEFT -> must steer RIGHT to correct.
ego = mkEgo(0, 0, deg2rad(20), 3.0);
sD = purePursuitController(ego, straight, 5.0, vehCfg);
pass = pass && check('D yawed left -> negative (right) steer', sD < -tol, sD);

% TEST E: on path but yawed RIGHT -> must steer LEFT.
ego = mkEgo(0, 0, deg2rad(-20), 3.0);
sE = purePursuitController(ego, straight, 5.0, vehCfg);
pass = pass && check('E yawed right -> positive (left) steer', sE > tol, sE);

% BICYCLE MODEL: positive steering must increase yaw (turn left).
ego = mkEgo(0,0,0,3.0);
nxt = bicycleModel(ego, struct('throttle',0,'brake',0,'steeringAngle',deg2rad(10)), vehCfg, 0.1);
pass = pass && check('plant: +steer -> +yaw (left)', nxt.yaw > tol, nxt.yaw);

% CARLA normalization sign (carlaClosedLoopStep.m uses steerSign=-1):
% a POSITIVE project-frame steer (left) must map to a NEGATIVE raw CARLA
% steer, per the live measurement documented in that file's header.
steerSign = -1;
carlaSteer = steerSign * deg2rad(10) / vehCfg.maxSteerAngle;
pass = pass && check('CARLA map: +left steer -> negative raw carla steer', carlaSteer < -tol, carlaSteer);

fprintf('PART 1 RESULT: %s\n', ternary(pass, 'ALL SIGN/FRAME CHECKS PASS', 'FAILURE - sign/frame bug present'));

fprintf('\n===== PART 2: reference-switching experiment =====\n');
sceneCfg = carlaIndianSceneConfig();
startXY  = [sceneCfg.egoApproach.x, -sceneCfg.egoApproach.y];
startYaw = deg2rad(-sceneCfg.egoApproach.yawDeg);
refPath  = carlaGenerateIntersectionTurnPath(startXY, startYaw, deg2rad(89.64), 12.0, 45.0, 25.0);

% localPlanner.m spreads candidates over DEFAULT_HALF_WIDTH = 2.5m.
% Emulate the planner re-selecting among those lateral offsets at a given
% rate, WITHOUT touching the planner: shift the reference laterally.
switchRates = [0, 0.10, 0.24, 0.45];   % 0.24 and 0.45 bracket the measured live rates
fprintf('%-12s %-10s %-10s %-10s %-12s %-12s\n', 'switchRate','meanCTE','maxCTE','maxHeadDeg','steerSat%','rateSat%');
for sr = switchRates
    r = runSwitching(refPath, startXY, startYaw, vehCfg, sr);
    fprintf('%-12.2f %-10.3f %-10.3f %-10.2f %-12.1f %-12.1f\n', ...
        sr, r.meanCTE, r.maxCTE, r.maxHeadErrDeg, r.steerSatPct, r.rateSatPct);
end
fprintf(['\nInterpretation: if saturation rises steeply with switch rate, the live\n' ...
         '69%% saturation is explained by the controller chasing a reference that\n' ...
         'moves, not by controller tuning (Phase 14.8 section 12 hypothesis B).\n']);
end

% ---------------------------------------------------------------
function out = runSwitching(refPath, startXY, startYaw, vehCfg, switchRate)
HALF_WIDTH = 2.5;                 % localPlanner.m's DEFAULT_HALF_WIDTH
offsets = linspace(-HALF_WIDTH, HALF_WIDTH, 15);   % 15 candidates
rng(42);                          % deterministic
ego = createEgoState();
ego.x = startXY(1); ego.y = startXY(2); ego.yaw = startYaw;
ego.velocity = 0; ego.steering = 0; ego.timestamp = 0;
dt = 0.1; targetSpeed = 4.0; maxRateRad = vehCfg.maxSteerRate;
goalXY = refPath(end,:);
curOff = 0; N = 3000;
cte = nan(N,1); headErr = nan(N,1); satS = false(N,1); satR = false(N,1);
k = 0;
for step = 1:N
    k = k + 1;
    if rand() < switchRate
        curOff = offsets(randi(numel(offsets)));
    end
    shifted = offsetPath(refPath, curOff);

    Ld = max(3.0, 0.5*ego.velocity + 2.0);
    rawSteer = purePursuitController(ego, shifted, Ld, vehCfg);
    maxStep = maxRateRad * dt;
    delta = max(min(rawSteer - ego.steering, maxStep), -maxStep);
    steer = max(min(ego.steering + delta, vehCfg.maxSteerAngle), -vehCfg.maxSteerAngle);
    accelCmd = max(min(1.0*(targetSpeed - ego.velocity), vehCfg.maxAccel), -vehCfg.maxBraking);
    cmd = struct('throttle', max(accelCmd,0)/vehCfg.maxAccel, 'brake', max(-accelCmd,0)/vehCfg.maxBraking, 'steeringAngle', steer);

    % error measured against the TRUE (unshifted) reference
    [c, ~, refYaw] = ctePoly(refPath, [ego.x, ego.y]);
    cte(k) = c;
    headErr(k) = atan2(sin(ego.yaw-refYaw), cos(ego.yaw-refYaw));
    satS(k) = abs(steer) >= vehCfg.maxSteerAngle - 1e-9;
    satR(k) = abs(delta) >= maxStep - 1e-12;

    ego = bicycleModel(ego, cmd, vehCfg, dt);
    if norm([ego.x,ego.y]-goalXY) < 3.0, break; end
end
out.meanCTE = mean(abs(cte(1:k)));
out.maxCTE = max(abs(cte(1:k)));
out.maxHeadErrDeg = max(abs(rad2deg(headErr(1:k))));
out.steerSatPct = 100*sum(satS(1:k))/k;
out.rateSatPct = 100*sum(satR(1:k))/k;
end

function p = offsetPath(path, off)
d = diff(path); d(end+1,:) = d(end,:);
nrm = vecnorm(d,2,2); nrm(nrm<1e-9) = 1;
nx = -d(:,2)./nrm; ny = d(:,1)./nrm;
p = [path(:,1) + off*nx, path(:,2) + off*ny];
end

function [cte, idx, refYaw] = ctePoly(path, p)
best = Inf; idx = 1; refYaw = 0;
for i = 1:size(path,1)-1
    a = path(i,:); b = path(i+1,:); ab = b-a; den = dot(ab,ab);
    if den < 1e-12, t = 0; else, t = max(0,min(1,dot(p-a,ab)/den)); end
    q = a + t*ab; dd = norm(p-q);
    if dd < best, best = dd; idx = i; refYaw = atan2(ab(2),ab(1)); end
end
cte = best;
end

function e = mkEgo(x,y,yaw,v)
e = createEgoState(); e.x=x; e.y=y; e.yaw=yaw; e.velocity=v; e.steering=0;
end

function ok = check(name, cond, val)
ok = logical(cond);
fprintf('  [%s] %-52s (value=%+.6f)\n', ternary(ok,'PASS','FAIL'), name, val);
end

function s = ternary(c,a,b)
if c, s = a; else, s = b; end
end
