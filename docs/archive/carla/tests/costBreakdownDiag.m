function tbl = costBreakdownDiag(egoState, candidateTrajectories, predictedTrajectories, vehicleConfig, plannerConfig, previousIndex, precomputedMinTTC, precomputedIsColliding)
% costBreakdownDiag - Phase 14.10 DIAGNOSTIC ONLY (tests/, never called by
% the pipeline). Reproduces planning/adaptivePlanner.m's per-candidate
% cost breakdown for live logging, WITHOUT re-invoking collisionCheck -
% carlaClosedLoopStep.m already computes candidateMinTTC/candidateColliding
% for every candidate each tick (it re-runs collisionCheck itself, exactly
% mirroring what adaptivePlanner.m does internally, for its own Phase 13
% diagnostics). Reusing those numbers here means this logger adds ZERO
% extra collisionCheck calls to the live loop - only the missing pieces
% (minClearance, uncertainty exposure, curvature, and the six-plus-one
% cost terms) are computed, mirroring adaptivePlanner.m's own arithmetic
% verbatim.
%
% Verified (see testPhase1410LiveAudit.m's startup check) to reproduce
% the exact same selection adaptivePlanner.m makes, given the SAME
% precomputed minTTC/isColliding - i.e. this is read-only instrumentation,
% not a second decision-making implementation.
%
% Output: a table with one row per candidate.

numCandidates = numel(candidateTrajectories);
centerline = candidateTrajectories{ceil(numCandidates / 2)};
w = plannerConfig.costWeights;
CONSISTENCY_WEIGHT = 0.3; % verbatim from adaptivePlanner.m

minClearances = Inf(numCandidates, 1);
curvatures = zeros(numCandidates, 1);
riskCosts = zeros(numCandidates, 1);
clearanceCosts = zeros(numCandidates, 1);
deviationCosts = zeros(numCandidates, 1);
curvatureCosts = zeros(numCandidates, 1);
speedChangeCosts = zeros(numCandidates, 1);
uncertaintyCosts = zeros(numCandidates, 1);
consistencyCosts = zeros(numCandidates, 1);
totalCosts = zeros(numCandidates, 1);
lateralOffsets = zeros(numCandidates, 1);
headingStarts = zeros(numCandidates, 1);

for c = 1:numCandidates
    candidate = candidateTrajectories{c};
    minTTC = precomputedMinTTC(c);

    [minClearance, uncertaintyExposure] = assessCandidateAgainstPredictionsDiag(candidate, predictedTrajectories);

    heading = atan2(diff(candidate(:, 2)), diff(candidate(:, 1)));
    segLengths = hypot(diff(candidate(:, 1)), diff(candidate(:, 2)));
    validSeg = segLengths > 1e-6;
    if any(validSeg), avgSegLength = mean(segLengths(validSeg)); else, avgSegLength = 1e-3; end
    maxHeadingChange = max(abs(diff(heading)));
    approxCurvature = maxHeadingChange / max(avgSegLength, 1e-3);
    impliedSafeSpeed = min(vehicleConfig.maxSpeed, sqrt(vehicleConfig.maxAccel / max(approxCurvature, 1e-3)));

    minClearances(c) = minClearance;
    curvatures(c) = approxCurvature;
    if ~isempty(heading), headingStarts(c) = heading(1); end

    % signed lateral offset from the centerline candidate, at the shared
    % nearest point - positive = left of centerline (ego-forward sense)
    m = min(size(candidate,1), size(centerline,1));
    d = candidate(1:m,:) - centerline(1:m,:);
    if m >= 2
        tang = centerline(2,:) - centerline(1,:);
        tang = tang / max(norm(tang), 1e-9);
        nrm = [-tang(2), tang(1)];
        lateralOffsets(c) = mean(d(:,1)*nrm(1) + d(:,2)*nrm(2));
    end

    riskCosts(c) = w.collisionRisk / max(minTTC, 0.1);
    clearanceCosts(c) = w.obstacleClearance / max(minClearance, 0.1);
    deviationCosts(c) = w.pathDeviation * mean(vecnorm(candidate - centerline, 2, 2));
    curvatureCosts(c) = w.curvature * sum(abs(diff(heading)));
    speedChangeCosts(c) = w.speedChange * abs(egoState.velocity - impliedSafeSpeed);
    uncertaintyCosts(c) = w.uncertainty * uncertaintyExposure;
    consistencyCosts(c) = CONSISTENCY_WEIGHT * abs(c - previousIndex) / numCandidates;

    totalCosts(c) = riskCosts(c) + clearanceCosts(c) + deviationCosts(c) + curvatureCosts(c) ...
                  + speedChangeCosts(c) + uncertaintyCosts(c) + consistencyCosts(c);
end

isFeasibleSafe = ~precomputedIsColliding(:) & precomputedMinTTC(:) >= plannerConfig.ttcThresholds.critical;
ttcRejected = ~precomputedIsColliding(:) & precomputedMinTTC(:) < plannerConfig.ttcThresholds.critical;

tbl = table((1:numCandidates)', lateralOffsets, rad2deg(headingStarts), curvatures, ...
    isFeasibleSafe, precomputedIsColliding(:), ttcRejected, precomputedMinTTC(:), minClearances, ...
    totalCosts, riskCosts, clearanceCosts, consistencyCosts, deviationCosts, curvatureCosts, ...
    speedChangeCosts, uncertaintyCosts, ...
    'VariableNames', {'candIdx','lateralOffset','headingStartDeg','curvature', ...
    'feasible','collisionRejected','ttcRejected','minTTC','minClearance', ...
    'totalCost','riskCost','clearanceCost','consistencyCost','deviationCost','curvatureCost', ...
    'speedChangeCost','uncertaintyCost'});
end

function [minClearance, uncertaintyExposure] = assessCandidateAgainstPredictionsDiag(candidate, predictedTrajectories)
% Verbatim copy of adaptivePlanner.m's local function of the same purpose.
minClearance = Inf; totalExposure = 0; exposureCount = 0;
for i = 1:numel(predictedTrajectories)
    pred = predictedTrajectories{i};
    if isempty(pred), continue; end
    numSteps = min(size(candidate, 1), size(pred, 1));
    for k = 1:numSteps
        dist = hypot(candidate(k, 1) - pred(k, 1), candidate(k, 2) - pred(k, 2));
        minClearance = min(minClearance, dist);
        totalExposure = totalExposure + pred(k, 3) / max(dist, 0.5);
        exposureCount = exposureCount + 1;
    end
end
if exposureCount > 0, uncertaintyExposure = totalExposure / exposureCount; else, uncertaintyExposure = 0; end
end
