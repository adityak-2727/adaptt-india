function testPhase149PlannerStability()
% testPhase149PlannerStability - Phase 14.9 sections 11/12/13/14.
%
% Offline, CARLA-free audit of adaptivePlanner's reference stability.
% Modifies NO production file. adaptivePlannerDiag.m (tests/) is a
% parameterized diagnostic COPY used only to answer questions the frozen
% implementation cannot be asked directly:
%   - what would be selected if the consistency term were absent
%   - what would be selected if it were stronger
%   - what would K2 select if consistency were applied there too
% It is verified against the real adaptivePlanner at default settings
% before any conclusion is drawn from it.

addpath(genpath(fileparts(fileparts(mfilename('fullpath')))));
vehCfg = vehicleConfig();
planCfg = plannerConfig();
sc = carlaIndianSceneConfig();

startXY = [sc.egoApproach.x, -sc.egoApproach.y];
startYaw = deg2rad(-sc.egoApproach.yawDeg);
refPath = carlaGenerateIntersectionTurnPath(startXY, startYaw, deg2rad(89.64), 12.0, 45.0, 25.0);

fprintf('=== Phase 14.9 planner reference-stability audit ===\n');

% ---- consistency-cost magnitude analysis (static, from the code) ----
CW = 0.3; n = planCfg.numCandidateTrajectories;
fprintf('\n--- consistency cost magnitude (CONSISTENCY_WEIGHT=%.1f, n=%d) ---\n', CW, n);
fprintf('  adjacent switch (delta=1) : %.4f\n', CW*1/n);
fprintf('  half-lattice   (delta=7) : %.4f\n', CW*7/n);
fprintf('  max            (delta=14): %.4f\n', CW*14/n);
fprintf('  riskCost 5.0/minTTC at minTTC=2.0 -> %.3f ; at 1.5 -> %.3f ; delta=%.3f\n', 5/2, 5/1.5, 5/1.5-5/2);
fprintf('  => a 0.5s minTTC change outweighs an adjacent-index switch by %.0fx\n', (5/1.5-5/2)/(CW*1/n));

% ---- equivalence check: diagnostic copy vs production ----
fprintf('\n--- diagnostic-copy equivalence vs production adaptivePlanner ---\n');
rng(7);
mismatches = 0; checks = 0;
for trial = 1:40
    [cands, preds, ego] = makeScenario(refPath, vehCfg, trial);
    prevIdx = randi(numel(cands));
    [~, iProd] = adaptivePlanner(ego, cands, preds, vehCfg, planCfg, "audit", prevIdx);
    [~, iDiag] = adaptivePlannerDiag(ego, cands, preds, vehCfg, planCfg, prevIdx, ...
        struct('consistencyWeight', 0.3, 'consistencyInK2', false));
    checks = checks + 1;
    if iProd ~= iDiag, mismatches = mismatches + 1; end
end
fprintf('  %d/%d identical selections (mismatches=%d)\n', checks-mismatches, checks, mismatches);
if mismatches > 0
    fprintf('  *** diagnostic copy is NOT equivalent - results below are invalid ***\n');
    return;
end

% ---- section 12: does consistency actually change decisions? ----
fprintf('\n--- section 12: consistency effectiveness (200 ticks) ---\n');
variants = { ...
    'production (w=0.3, K2 no consistency)', struct('consistencyWeight',0.3,'consistencyInK2',false); ...
    'consistency OFF (w=0)',                 struct('consistencyWeight',0.0,'consistencyInK2',false); ...
    'w=1.5 (5x)',                            struct('consistencyWeight',1.5,'consistencyInK2',false); ...
    'w=3.0 (10x)',                           struct('consistencyWeight',3.0,'consistencyInK2',false); ...
    'w=0.3 + consistency IN K2',             struct('consistencyWeight',0.3,'consistencyInK2',true); ...
    'w=3.0 + consistency IN K2',             struct('consistencyWeight',3.0,'consistencyInK2',true)};

fprintf('%-38s %-10s %-12s %-12s %-10s\n','variant','switch%','meanLatMove','maxLatMove','K2%');
for v = 1:size(variants,1)
    r = runSequence(refPath, vehCfg, planCfg, variants{v,2});
    fprintf('%-38s %-10.1f %-12.3f %-12.3f %-10.1f\n', variants{v,1}, r.switchPct, r.meanLat, r.maxLat, r.k2Pct);
end

fprintf(['\nmeanLatMove/maxLatMove are the mean/max lateral displacement of the\n' ...
         'SELECTED reference between consecutive ticks - what the controller\n' ...
         'actually has to chase (Phase 14.9 section 8).\n']);
end

% =====================================================================
function out = runSequence(refPath, vehCfg, planCfg, opts)
% Walk the ego along the reference while jittering predicted obstacles,
% and record how the selected reference moves between ticks.
rng(11);
NT = 200;
prevIdx = [];
prevSel = [];
switches = 0; lat = []; k2n = 0; n = 0;
for t = 1:NT
    [cands, preds, ego] = makeScenario(refPath, vehCfg, t);
    [sel, idx, usedK2] = adaptivePlannerDiag(ego, cands, preds, vehCfg, planCfg, prevIdx, opts);
    n = n + 1;
    if usedK2, k2n = k2n + 1; end
    if ~isempty(prevIdx) && idx ~= prevIdx
        switches = switches + 1;
        m = min(size(sel,1), size(prevSel,1));
        lat(end+1) = mean(vecnorm(sel(1:m,:) - prevSel(1:m,:), 2, 2)); %#ok<AGROW>
    end
    prevIdx = idx; prevSel = sel;
end
out.switchPct = 100*switches/max(n-1,1);
if isempty(lat), out.meanLat = 0; out.maxLat = 0;
else, out.meanLat = mean(lat); out.maxLat = max(lat); end
out.k2Pct = 100*k2n/n;
end

% =====================================================================
function [cands, preds, ego] = makeScenario(refPath, vehCfg, t)
% Ego progresses along the reference; two predicted agents sit near the
% corridor and jitter tick to tick, emulating tracker noise on real
% obstacles. Candidates come from the REAL frozen localPlanner.
planCfg = plannerConfig();
idx = min(1 + mod(t*2, max(size(refPath,1)-30,1)), size(refPath,1)-2);
ego = createEgoState();
ego.x = refPath(idx,1); ego.y = refPath(idx,2);
d = refPath(idx+1,:) - refPath(idx,:);
ego.yaw = atan2(d(2), d(1));
ego.velocity = 2.5; ego.steering = 0; ego.timestamp = t*0.1;

% two agents offset laterally from the path ahead of the ego, with
% small per-tick jitter (the "planner input noise" of section 18)
preds = {};
for a = 1:2
    % Placed at the lateral offsets Phase 14.6 forensics measured for the
    % obstacles that actually rejected candidates live (1.7-2.6m from the
    % corridor), with tracker-scale jitter. Farther-out obstacles reject
    % nothing and produce a degenerate 0%-switching scenario.
    ai = min(idx + 8 + 5*a, size(refPath,1));
    base = refPath(ai,:);
    nrm = [-d(2), d(1)] / max(norm(d),1e-9);
    side = (-1)^a * (1.7 + 0.35*sin(t*0.7 + a));
    jitter = 0.25 * [sin(t*1.3 + a), cos(t*1.1 + a)];
    p0 = base + side*nrm + jitter;
    steps = 41;
    traj = zeros(steps,3);
    for k = 1:steps
        traj(k,1:2) = p0 + (k-1)*0.1*[0.3*(-1)^a, 0.15];
        traj(k,3) = 0.3 + 0.02*k;   % uncertainty radius
    end
    preds{end+1} = traj; %#ok<AGROW>
end

cands = localPlanner(ego, refPath, preds, planCfg);
end
