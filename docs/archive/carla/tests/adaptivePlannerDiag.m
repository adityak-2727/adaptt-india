function [selectedTrajectory, selectedIndex, usedFallback] = adaptivePlannerDiag(egoState, candidateTrajectories, predictedTrajectories, vehicleConfig, plannerConfig, previousIndex, opts)
% adaptivePlannerDiag - Phase 14.9 DIAGNOSTIC COPY of planning/adaptivePlanner.m.
%
% NOT production code. Lives in tests/ and is never called by the pipeline.
% It exists because two audit questions cannot be asked of the frozen
% implementation from outside:
%   1. what would be selected if the consistency term were absent/stronger
%   2. what would K2 select if the consistency term were applied there too
%
% Every cost term, threshold, and the fallback ranking are mirrored
% VERBATIM from planning/adaptivePlanner.m. The only differences are the
% two knobs in `opts`. testPhase149PlannerStability.m asserts this copy
% reproduces the real planner's selection exactly at default settings
% before drawing any conclusion from it.
%
% opts.consistencyWeight - replaces the frozen CONSISTENCY_WEIGHT (0.3)
% opts.consistencyInK2   - when true, the K2 fallback ranking also gets a
%                          consistency preference (the frozen version has
%                          none - it ranks purely by minTTC then
%                          minClearance)

if nargin < 7 || isempty(opts)
    opts = struct();
end
if ~isfield(opts, 'consistencyWeight'), opts.consistencyWeight = 0.3; end
if ~isfield(opts, 'consistencyInK2'),   opts.consistencyInK2 = false; end

usedFallback = false;

if isempty(candidateTrajectories)
    selectedTrajectory = zeros(0, 2); selectedIndex = []; return;
end

numCandidates = numel(candidateTrajectories);
centerline = candidateTrajectories{ceil(numCandidates / 2)};
w = plannerConfig.costWeights;

if nargin < 6 || isempty(previousIndex)
    previousIndex = ceil(numCandidates / 2);
end

CONSISTENCY_WEIGHT = opts.consistencyWeight;

costs = zeros(numCandidates, 1);
minTTCs = Inf(numCandidates, 1);
isCollidingFlags = false(numCandidates, 1);
minClearances = Inf(numCandidates, 1);
curvatures = zeros(numCandidates, 1);

for c = 1:numCandidates
    candidate = candidateTrajectories{c};
    [isColliding, minTTC] = collisionCheck(candidate, predictedTrajectories, vehicleConfig);
    isCollidingFlags(c) = isColliding;
    minTTCs(c) = minTTC;

    [minClearance, uncertaintyExposure] = assessCandidateAgainstPredictions(candidate, predictedTrajectories);

    heading = atan2(diff(candidate(:, 2)), diff(candidate(:, 1)));
    segLengths = hypot(diff(candidate(:, 1)), diff(candidate(:, 2)));
    validSeg = segLengths > 1e-6;
    if any(validSeg), avgSegLength = mean(segLengths(validSeg)); else, avgSegLength = 1e-3; end
    maxHeadingChange = max(abs(diff(heading)));
    approxCurvature = maxHeadingChange / max(avgSegLength, 1e-3);
    impliedSafeSpeed = min(vehicleConfig.maxSpeed, sqrt(vehicleConfig.maxAccel / max(approxCurvature, 1e-3)));

    minClearances(c) = minClearance;
    curvatures(c) = approxCurvature;

    riskCost = w.collisionRisk / max(minTTC, 0.1);
    clearanceCost = w.obstacleClearance / max(minClearance, 0.1);
    deviationCost = w.pathDeviation * mean(vecnorm(candidate - centerline, 2, 2));
    curvatureCost = w.curvature * sum(abs(diff(heading)));
    speedChangeCost = w.speedChange * abs(egoState.velocity - impliedSafeSpeed);
    uncertaintyCost = w.uncertainty * uncertaintyExposure;
    consistencyCost = CONSISTENCY_WEIGHT * abs(c - previousIndex) / numCandidates;

    costs(c) = riskCost + clearanceCost + deviationCost + curvatureCost + speedChangeCost + uncertaintyCost + consistencyCost;
end

safeIdx = find(~isCollidingFlags & minTTCs >= plannerConfig.ttcThresholds.critical);
if ~isempty(safeIdx)
    [~, bestLocal] = min(costs(safeIdx));
    bestIdx = safeIdx(bestLocal);
else
    usedFallback = true;
    maxFeasibleCurvature = tan(vehicleConfig.maxSteerAngle) / vehicleConfig.wheelbase;
    isFeasible = curvatures <= maxFeasibleCurvature;
    feasibleIdx = find(isFeasible);
    if ~isempty(feasibleIdx)
        if opts.consistencyInK2
            % DIAGNOSTIC ONLY. Safety ordering is unchanged - minTTC is
            % still primary; consistency only breaks ties among candidates
            % whose minTTC is within a small epsilon, so it can never
            % promote a less-safe candidate over a safer one.
            ttcs = minTTCs(feasibleIdx);
            clrs = minClearances(feasibleIdx);
            prox = abs(feasibleIdx(:) - previousIndex) / numCandidates;
            EPS = 0.05;
            bucket = -floor(ttcs / EPS);          % coarser = ties within EPS seconds
            rankKeys = [bucket, prox, -clrs];
            [~, order] = sortrows(rankKeys, [1, 2, 3]);
            bestIdx = feasibleIdx(order(1));
        else
            rankKeys = [minTTCs(feasibleIdx), minClearances(feasibleIdx)];
            [~, order] = sortrows(rankKeys, [-1, -2]);
            bestIdx = feasibleIdx(order(1));
        end
    else
        [~, bestIdx] = max(minTTCs);
    end
end

selectedTrajectory = candidateTrajectories{bestIdx};
selectedIndex = bestIdx;
end

function [minClearance, uncertaintyExposure] = assessCandidateAgainstPredictions(candidate, predictedTrajectories)
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
