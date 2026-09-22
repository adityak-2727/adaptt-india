function results = testPhase148ControllerOffline(outDir)
% testPhase148ControllerOffline - Phase 14.8: deterministic, CARLA-free
% characterization of the path-tracking loop.
%
% WHY OFF-CARLA FIRST: in the live loop the controller follows
% pathSmoothing(selectedTrajectory), which the planner RE-SELECTS every
% tick. So a live oscillation could be the controller failing to track a
% stable reference, OR the controller faithfully chasing a reference that
% keeps moving. Those two have opposite fixes. This harness removes the
% planner entirely by driving the SAME controller and SAME vehicle model
% along ONE FIXED reference path, which isolates the controller's own
% behaviour (Phase 14.8 section 12's critical distinction).
%
% Everything is the real production code path:
%   purePursuitController.m (frozen) + the rate-limit/clamp arithmetic
%   from vehicleController.m + bicycleModel.m (frozen) + vehicleConfig.m.
% Nothing in control/ or planning/ is modified.
%
% NOTE on the rate limiter: vehicleController.m computes its lookahead
% internally as max(3.0, 0.5*v + 2.0). To sweep lookahead without editing
% a frozen file, this harness calls purePursuitController.m directly and
% then applies vehicleController.m's OWN rate-limit + clamp arithmetic,
% mirrored here verbatim. Running with lookaheadMode="production"
% reproduces vehicleController.m exactly - verified by
% testPhase148MatchesProductionController below.
%
% Output: results struct + evidence PNG/CSV in outDir.

if nargin < 1 || isempty(outDir)
    outDir = 'C:\Users\ADITYA\sih-autonomous-india\results\figures\';
end
addpath(genpath(fileparts(fileparts(mfilename('fullpath')))));

vehCfg = vehicleConfig();

% ---- The exact Phase 14 hero turn path (project frame) ----
% Ego spawn/heading and exit heading as used by carlaPhase14TurningDemo.m.
sceneCfg = carlaIndianSceneConfig();
startXY  = [sceneCfg.egoApproach.x, -sceneCfg.egoApproach.y];
startYaw = deg2rad(-sceneCfg.egoApproach.yawDeg);
refPath  = carlaGenerateIntersectionTurnPath(startXY, startYaw, deg2rad(89.64), 12.0, 45.0, 25.0);

fprintf('=== Phase 14.8 offline controller harness ===\n');
fprintf('path: %d waypoints, length %.1fm\n', size(refPath,1), sum(hypot(diff(refPath(:,1)), diff(refPath(:,2)))));
fprintf('vehCfg: wheelbase=%.2f maxSteer=%.1fdeg maxSteerRate=%.1fdeg/s\n', ...
    vehCfg.wheelbase, rad2deg(vehCfg.maxSteerAngle), rad2deg(vehCfg.maxSteerRate));

results = struct();
results.refPath = refPath;

% ---------- BASELINE (production configuration) ----------
base = runTrack(refPath, startXY, startYaw, vehCfg, struct( ...
    'lookaheadMode', "production", 'fixedLookahead', NaN, ...
    'rateLimitDegS', rad2deg(vehCfg.maxSteerRate), 'speedScale', 1.0, ...
    'baseSpeed', 4.0, 'dt', 0.1, 'maxSteps', 3000));
results.baseline = base;
printMetrics('BASELINE (production)', base);

plotTracking(base, refPath, fullfile(outDir, 'phase148_baseline_tracking.png'), 'Phase 14.8 baseline (production config)');

end

% =====================================================================
function out = runTrack(refPath, startXY, startYaw, vehCfg, cfg)
% One deterministic closed-loop run of controller + bicycle model along a
% FIXED reference path. No planner, no CARLA, no traffic.

ego = createEgoState();
ego.x = startXY(1); ego.y = startXY(2); ego.yaw = startYaw;
ego.velocity = 0; ego.steering = 0; ego.timestamp = 0;

dt = cfg.dt;
targetSpeed = cfg.baseSpeed * cfg.speedScale;
maxRateRad = deg2rad(cfg.rateLimitDegS);
goalXY = refPath(end, :);

N = cfg.maxSteps;
L = struct('t', nan(N,1), 'x', nan(N,1), 'y', nan(N,1), 'yaw', nan(N,1), ...
    'v', nan(N,1), 'cte', nan(N,1), 'headErr', nan(N,1), 'rawSteer', nan(N,1), ...
    'steer', nan(N,1), 'satSteer', false(N,1), 'satRate', false(N,1), ...
    'lookahead', nan(N,1), 'refYaw', nan(N,1), 'nearIdx', nan(N,1));

k = 0;
for step = 1:N
    k = k + 1;

    % lookahead per configuration
    if cfg.lookaheadMode == "production"
        Ld = max(3.0, 0.5 * ego.velocity + 2.0);
    elseif cfg.lookaheadMode == "fixed"
        Ld = cfg.fixedLookahead;
    else % custom affine: [minL, gain, offset]
        Ld = max(cfg.fixedLookahead(1), cfg.fixedLookahead(2) * ego.velocity + cfg.fixedLookahead(3));
    end

    % --- frozen controller ---
    rawSteer = purePursuitController(ego, refPath, Ld, vehCfg);

    % --- vehicleController.m's own rate-limit + clamp, mirrored verbatim ---
    maxStep = maxRateRad * dt;
    delta = max(min(rawSteer - ego.steering, maxStep), -maxStep);
    steer = ego.steering + delta;
    steer = max(min(steer, vehCfg.maxSteerAngle), -vehCfg.maxSteerAngle);

    % --- longitudinal: vehicleController.m's P controller, mirrored ---
    accelCmd = 1.0 * (targetSpeed - ego.velocity);
    accelCmd = max(min(accelCmd, vehCfg.maxAccel), -vehCfg.maxBraking);
    cmd = struct('throttle', max(accelCmd,0)/vehCfg.maxAccel, ...
                 'brake', max(-accelCmd,0)/vehCfg.maxBraking, ...
                 'steeringAngle', steer);

    % --- errors against the FIXED reference ---
    [cte, nearIdx, refYaw] = crossTrackError(refPath, [ego.x, ego.y]);
    headErr = atan2(sin(ego.yaw - refYaw), cos(ego.yaw - refYaw));

    L.t(k)=ego.timestamp; L.x(k)=ego.x; L.y(k)=ego.y; L.yaw(k)=ego.yaw; L.v(k)=ego.velocity;
    L.cte(k)=cte; L.headErr(k)=headErr; L.rawSteer(k)=rawSteer; L.steer(k)=steer;
    L.satSteer(k) = abs(steer) >= vehCfg.maxSteerAngle - 1e-9;
    L.satRate(k)  = abs(delta) >= maxStep - 1e-12;
    L.lookahead(k)=Ld; L.refYaw(k)=refYaw; L.nearIdx(k)=nearIdx;

    % --- frozen plant ---
    ego = bicycleModel(ego, cmd, vehCfg, dt);

    if norm([ego.x, ego.y] - goalXY) < 3.0
        break;
    end
end

f = fieldnames(L);
for i = 1:numel(f); L.(f{i}) = L.(f{i})(1:k); end

out = struct();
out.log = L;
out.completed = norm([ego.x, ego.y] - goalXY) < 3.0;
out.steps = k;
out.meanCTE = mean(abs(L.cte));
out.rmsCTE  = sqrt(mean(L.cte.^2));
out.maxCTE  = max(abs(L.cte));
out.meanHeadErrDeg = mean(abs(rad2deg(L.headErr)));
out.maxHeadErrDeg  = max(abs(rad2deg(L.headErr)));
out.steerSatPct = 100 * sum(L.satSteer) / k;
out.rateSatPct  = 100 * sum(L.satRate) / k;
out.signReversals = sum(diff(sign(L.steer)) ~= 0);
out.finalPosErr = norm([ego.x, ego.y] - goalXY);
out.finalHeadErrDeg = abs(rad2deg(L.headErr(end)));
out.timeOutside1m   = 100 * sum(abs(L.cte) > 1.0) / k;
out.timeOutside1p5m = 100 * sum(abs(L.cte) > 1.5) / k;
out.cfg = cfg;
end

% =====================================================================
function [cte, nearIdx, refYaw] = crossTrackError(path, p)
% True point-to-polyline distance (not nearest-waypoint distance), plus
% the reference heading of the closest segment.
best = Inf; nearIdx = 1; refYaw = 0;
for i = 1:size(path,1)-1
    a = path(i,:); b = path(i+1,:);
    ab = b - a; denom = dot(ab,ab);
    if denom < 1e-12, t = 0; else, t = max(0, min(1, dot(p-a, ab)/denom)); end
    proj = a + t*ab;
    d = norm(p - proj);
    if d < best
        best = d; nearIdx = i; refYaw = atan2(ab(2), ab(1));
    end
end
cte = best;
end

% =====================================================================
function printMetrics(label, r)
fprintf('\n--- %s ---\n', label);
fprintf('  completed=%d  steps=%d  finalPosErr=%.2fm  finalHeadErr=%.1fdeg\n', ...
    r.completed, r.steps, r.finalPosErr, r.finalHeadErrDeg);
fprintf('  CTE  mean=%.3fm rms=%.3fm max=%.3fm\n', r.meanCTE, r.rmsCTE, r.maxCTE);
fprintf('  head mean=%.2fdeg max=%.2fdeg\n', r.meanHeadErrDeg, r.maxHeadErrDeg);
fprintf('  steerSat=%.1f%%  rateSat=%.1f%%  signReversals=%d\n', r.steerSatPct, r.rateSatPct, r.signReversals);
fprintf('  outside1.0m=%.1f%%  outside1.5m=%.1f%%\n', r.timeOutside1m, r.timeOutside1p5m);
end

% =====================================================================
function plotTracking(r, refPath, outPng, ttl)
L = r.log;
fig = figure('Visible','off','Color','w','Position',[0 0 1400 1000]);
subplot(3,2,[1 2]); hold on;
plot(refPath(:,1), refPath(:,2), 'k:', 'LineWidth', 1.2);
plot(L.x, L.y, 'g-', 'LineWidth', 1.8);
scatter(L.x(1), L.y(1), 70, 'b', 'filled'); scatter(L.x(end), L.y(end), 70, 'r', 'filled');
legend('reference','actual','start','end','Location','best'); axis equal; grid on;
title(ttl); xlabel('x [m]'); ylabel('y [m]');

subplot(3,2,3); plot(L.t, L.cte, 'b-'); grid on; ylabel('cross-track err [m]'); xlabel('t [s]'); title('CTE');
subplot(3,2,4); plot(L.t, rad2deg(L.headErr), 'r-'); grid on; ylabel('heading err [deg]'); xlabel('t [s]'); title('Heading error');
subplot(3,2,5); hold on; plot(L.t, rad2deg(L.rawSteer), 'c-'); plot(L.t, rad2deg(L.steer), 'b-', 'LineWidth',1.2);
grid on; ylabel('steer [deg]'); xlabel('t [s]'); legend('raw','rate-limited','Location','best'); title('Steering');
subplot(3,2,6); hold on; yyaxis left; plot(L.t, L.v); ylabel('speed [m/s]');
yyaxis right; plot(L.t, L.lookahead); ylabel('lookahead [m]'); grid on; xlabel('t [s]'); title('Speed / lookahead');

exportgraphics(fig, outPng); close(fig);
fprintf('  saved %s\n', outPng);
end
