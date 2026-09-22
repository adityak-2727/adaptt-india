classdef AutonomyPipelineBlock < matlab.System
    % AutonomyPipelineBlock - hosts the REAL, unmodified closed-loop
    % autonomy pipeline (perception -> fusion -> tracking -> prediction
    % (K1) -> decision -> planning (K2) -> collision check -> control ->
    % vehicle motion) as one Simulink block, so the pipeline genuinely
    % runs inside Simulink, not just alongside it.
    %
    % This block calls the exact same, unmodified function files main.m
    % calls, in the exact same order, every tick:
    %   cameraDetection, lidarDetection, radarDetection, sensorFusion,
    %   objectTracking, trajectoryPrediction (K1, frozen),
    %   behaviorDecision, decisionStateMachine, behaviorSeverity,
    %   localPlanner, adaptivePlanner (K2, frozen), collisionCheck,
    %   pathSmoothing, vehicleController, bicycleModel.
    % None of those files were modified to build this block - it is a
    % pure caller, exactly like demo/runDemo.m is a pure caller of
    % main.m. See planning/adaptivePlanner.m, prediction/
    % trajectoryPrediction.m etc. directly for what each stage actually
    % does; this block does not reimplement any of it.
    %
    % Two pieces are DUPLICATED here rather than shared, both
    % non-algorithmic plumbing (not planner/prediction/decision/control
    % logic), because they exist as LOCAL functions inside main.m
    % (dispatchScenario mirrors main.m's selectScenarioByName;
    % filterAgentsForSensor mirrors main.m's own same-named local
    % function) and MATLAB local functions cannot be called from another
    % file. Duplicating ~15 lines of sensor-FOV geometry and a name->
    % function switch was judged lower-risk than modifying main.m (which
    % is otherwise untouched by this work) to extract them. The two
    % speed-factor structs (scenario base-speed context, decision-state
    % speed multipliers) are likewise copied verbatim from main.m's own
    % literals, not derived independently.
    %
    % SimulateUsing must be 'Interpreted Execution' (set by
    % buildAutonomyPipelineModel.m) - this block's callees use struct
    % arrays and cell arrays throughout (tracked agents, predicted
    % trajectories, candidate trajectories), which are not
    % code-generation compatible.
    %
    % Closed-loop feedback: every property below that represents live
    % state (EgoState, GroundTruthAgents, TrackedAgentsPrev,
    % DecisionState, PreviousCandidateIndex, ...) is a persistent System
    % object property, updated at the end of each stepImpl call and read
    % again at the start of the next one - each Simulink tick is exactly
    % one iteration of main.m's for-loop body, with state genuinely
    % carried tick-to-tick the same way main.m's loop variables are
    % carried iteration-to-iteration. Once the scenario's goal is
    % reached, the block freezes (stops re-running the pipeline) and
    % holds the final state, mirroring main.m's `break`.

    properties (Nontunable)
        ScenarioName = "villageRoad" % one of the five existing scenarios/*.m names
    end

    properties (Access = private)
        SimCfg
        VehCfg
        PlanCfg
        SensorCfg
        Scenario
        EgoState
        GroundTruthAgents
        TrackedAgentsPrev
        DecisionState
        PlanHorizon
        GoalTolerance
        ContactDistance
        MinDwellTime
        LastTransitionTime
        PreviousCandidateIndex
        GlobalPath
        BaseSpeed
        DecisionSpeedFactors
        TickCount
        GoalReachedFlag
        LastSteerDeg
        LastThrottle
        LastBrake
        LastMinTTC
        LastIsColliding
        LastSelectedIndex
        LastNumTracked
        LastNumPredicted
        LastDecisionSeverity
    end

    methods (Access = protected)
        function setupImpl(obj)
            obj.SimCfg = simulationConfig();
            obj.VehCfg = vehicleConfig();
            obj.PlanCfg = plannerConfig();
            obj.SensorCfg = sensorConfig();

            rng(obj.SimCfg.randomSeed);

            obj.Scenario = obj.dispatchScenario(obj.ScenarioName);

            obj.EgoState = obj.Scenario.egoStart;
            obj.GroundTruthAgents = obj.Scenario.agents;
            obj.TrackedAgentsPrev = repmat(createAgent(), 0, 0);
            obj.DecisionState = "cruise";
            obj.PlanHorizon = 4.0;         % [s] matches main.m
            obj.GoalTolerance = 2.0;       % [m] matches main.m
            obj.ContactDistance = 1.1;     % [m] matches main.m / collisionCheck.m's egoRadius
            obj.MinDwellTime = 1.5;        % [s] matches main.m
            obj.LastTransitionTime = -Inf;
            obj.PreviousCandidateIndex = [];
            obj.TickCount = 0;
            obj.GoalReachedFlag = false;
            obj.LastSteerDeg = 0;
            obj.LastThrottle = 0;
            obj.LastBrake = 0;
            obj.LastMinTTC = Inf;
            obj.LastIsColliding = false;
            obj.LastSelectedIndex = 0;
            obj.LastNumTracked = 0;
            obj.LastNumPredicted = 0;
            obj.LastDecisionSeverity = behaviorSeverity(obj.DecisionState);

            startPose = [obj.EgoState.x, obj.EgoState.y, obj.EgoState.yaw];
            goalPose = [obj.Scenario.egoGoal(1), obj.Scenario.egoGoal(2), 0];
            obj.GlobalPath = globalPlanner(startPose, goalPose, obj.Scenario.map);

            % Copied verbatim from main.m's own scenarioSpeedFactors literal.
            scenarioSpeedFactors = struct( ...
                'villageRoad',       0.5, ...
                'urbanIntersection', 0.5, ...
                'highwayMerge',      0.65, ...
                'marketArea',        0.3, ...
                'cattleCrossing',    0.5 ...
            );
            if isfield(scenarioSpeedFactors, obj.Scenario.name)
                obj.BaseSpeed = scenarioSpeedFactors.(obj.Scenario.name) * obj.VehCfg.maxSpeed;
            else
                obj.BaseSpeed = 0.5 * obj.VehCfg.maxSpeed;
            end

            % Copied verbatim from main.m's own decisionSpeedFactors literal.
            obj.DecisionSpeedFactors = struct( ...
                'cruise',         1.0, ...
                'follow',         0.8, ...
                'merge',          0.8, ...
                'replan',         0.75, ...
                'avoid',          0.7, ...
                'brake',          0.15, ...
                'wait',           0.0, ...
                'emergency_stop', 0.0 ...
            );
        end

        function [egoX, egoY, egoYaw, egoVelocity, numTrackedAgents, numPredictedTrajectories, ...
                  decisionSeverityOut, selectedCandidateIndex, minTTCOut, isCollidingOut, ...
                  distToGoal, goalReachedOut, steeringDeg, throttleCmd, brakeCmd, tickCountOut] = stepImpl(obj)

            obj.TickCount = obj.TickCount + 1;

            if ~obj.GoalReachedFlag
                % --- 1. Perception ---
                cameraView = obj.filterAgentsForSensor(obj.GroundTruthAgents, obj.EgoState, obj.SensorCfg.camera.maxRange, obj.SensorCfg.camera.fov);
                lidarView  = obj.filterAgentsForSensor(obj.GroundTruthAgents, obj.EgoState, obj.SensorCfg.lidar.maxRange, obj.SensorCfg.lidar.fov);
                radarView  = obj.filterAgentsForSensor(obj.GroundTruthAgents, obj.EgoState, obj.SensorCfg.radar.maxRange, obj.SensorCfg.radar.fov);

                cameraAgents = cameraDetection(cameraView, obj.SensorCfg.camera, obj.EgoState.timestamp);
                lidarAgents  = lidarDetection(lidarView, obj.SensorCfg.lidar, obj.EgoState.timestamp);
                radarAgents  = radarDetection(radarView, obj.SensorCfg.radar, obj.EgoState.timestamp);
                fusedAgents  = sensorFusion(cameraAgents, lidarAgents, radarAgents, false, obj.TrackedAgentsPrev);

                % --- 2. Tracking ---
                trackedAgents = objectTracking(fusedAgents, obj.TrackedAgentsPrev, obj.SimCfg.dt);
                obj.TrackedAgentsPrev = trackedAgents;

                % --- 3. Prediction (K1, frozen) ---
                predictedTrajectories = trajectoryPrediction(trackedAgents, obj.EgoState, obj.PlanHorizon, obj.SimCfg.dt);

                % --- 4. Decision ---
                behaviorCommand = behaviorDecision(obj.EgoState, trackedAgents, predictedTrajectories, obj.PlanCfg, obj.DecisionState);
                proposedState = decisionStateMachine(obj.DecisionState, behaviorCommand, struct());
                isEscalation = behaviorSeverity(proposedState) > behaviorSeverity(obj.DecisionState);
                dwellSatisfied = (obj.EgoState.timestamp - obj.LastTransitionTime) >= obj.MinDwellTime;
                if proposedState ~= obj.DecisionState && (isEscalation || dwellSatisfied)
                    obj.DecisionState = proposedState;
                    obj.LastTransitionTime = obj.EgoState.timestamp;
                end
                targetSpeed = obj.BaseSpeed * obj.DecisionSpeedFactors.(obj.DecisionState);

                % --- 5. Planning (K2, frozen) + collision check ---
                candidateTrajectories = localPlanner(obj.EgoState, obj.GlobalPath, predictedTrajectories, obj.PlanCfg);
                priorCandidateIndex = obj.PreviousCandidateIndex;
                [selectedTrajectory, obj.PreviousCandidateIndex] = adaptivePlanner(obj.EgoState, candidateTrajectories, predictedTrajectories, obj.VehCfg, obj.PlanCfg, obj.Scenario.name, priorCandidateIndex);
                [isColliding, minTTC] = collisionCheck(selectedTrajectory, predictedTrajectories, obj.VehCfg);
                smoothPath = pathSmoothing(selectedTrajectory, struct());

                % --- 6. Control ---
                controlCommand = vehicleController(obj.EgoState, smoothPath, obj.VehCfg, targetSpeed, obj.SimCfg.dt);

                % --- 7. Vehicle model (closes the loop back to perception at t+1) ---
                obj.EgoState = bicycleModel(obj.EgoState, controlCommand, obj.VehCfg, obj.SimCfg.dt);

                % --- Ground-truth agent motion (perception above only observes a degraded view of this) ---
                for a = 1:numel(obj.GroundTruthAgents)
                    obj.GroundTruthAgents(a).position = obj.GroundTruthAgents(a).position + obj.GroundTruthAgents(a).velocity * obj.SimCfg.dt;
                    obj.GroundTruthAgents(a).timestamp = obj.EgoState.timestamp;
                end

                obj.LastSteerDeg = rad2deg(controlCommand.steeringAngle);
                obj.LastThrottle = controlCommand.throttle;
                obj.LastBrake = controlCommand.brake;
                obj.LastMinTTC = minTTC;
                obj.LastIsColliding = isColliding;
                obj.LastSelectedIndex = obj.PreviousCandidateIndex;
                obj.LastNumTracked = numel(trackedAgents);
                obj.LastNumPredicted = numel(predictedTrajectories);
                obj.LastDecisionSeverity = behaviorSeverity(obj.DecisionState);

                if norm([obj.EgoState.x, obj.EgoState.y] - obj.Scenario.egoGoal) < obj.GoalTolerance
                    obj.GoalReachedFlag = true;
                end
            end

            egoX = obj.EgoState.x;
            egoY = obj.EgoState.y;
            egoYaw = obj.EgoState.yaw;
            egoVelocity = obj.EgoState.velocity;
            numTrackedAgents = obj.LastNumTracked;
            numPredictedTrajectories = obj.LastNumPredicted;
            decisionSeverityOut = obj.LastDecisionSeverity;
            selectedCandidateIndex = obj.LastSelectedIndex;
            minTTCOut = obj.LastMinTTC;
            isCollidingOut = double(obj.LastIsColliding);
            distToGoal = norm([obj.EgoState.x, obj.EgoState.y] - obj.Scenario.egoGoal);
            goalReachedOut = double(obj.GoalReachedFlag);
            steeringDeg = obj.LastSteerDeg;
            throttleCmd = obj.LastThrottle;
            brakeCmd = obj.LastBrake;
            tickCountOut = obj.TickCount;
        end

        function resetImpl(~)
        end

        % Explicit output port property overrides - required so Simulink
        % can determine port size/type/complexity without running its
        % code-generation-based inference pass (which this block's
        % struct/cell-array-using code cannot support, even with
        % SimulateUsing='Interpreted Execution' - found live: without
        % these overrides, sim() fails at the "pre propagation phase"
        % with "Property 'Scenario' is undefined", since Simulink tries
        % to partially execute stepImpl through the code-gen path to
        % infer sizes before setupImpl has ever run). All 16 outputs are
        % fixed-size real scalars.
        function varargout = getOutputSizeImpl(~)
            varargout = repmat({1}, 1, 16);
        end

        function varargout = getOutputDataTypeImpl(~)
            varargout = repmat({'double'}, 1, 16);
        end

        function varargout = isOutputComplexImpl(~)
            varargout = repmat({false}, 1, 16);
        end

        function varargout = isOutputFixedSizeImpl(~)
            varargout = repmat({true}, 1, 16);
        end
    end

    methods (Access = private)
        function scenario = dispatchScenario(~, name)
            % Mirrors main.m's own local function selectScenarioByName -
            % duplicated because main.m's is a local function and cannot
            % be called from outside main.m. See this file's class-level
            % comment for why duplication (not modifying main.m) was
            % chosen here.
            name = string(name);
            switch name
                case "villageRoad"
                    scenario = villageRoad();
                case "urbanIntersection"
                    scenario = urbanIntersection();
                case "highwayMerge"
                    scenario = highwayMerge();
                case "marketArea"
                    scenario = marketArea();
                case "cattleCrossing"
                    scenario = cattleCrossing();
                otherwise
                    error('AutonomyPipelineBlock:invalidScenario', ...
                        '"%s" is not a supported scenario.', name);
            end
        end

        function inRangeAgents = filterAgentsForSensor(~, agents, egoState, maxRange, fov)
            % Mirrors main.m's own local function filterAgentsForSensor
            % verbatim - duplicated for the same reason as
            % dispatchScenario above (main.m's copy is a local function).
            inRangeAgents = repmat(createAgent(), 0, 0);
            egoPos = [egoState.x, egoState.y];
            for i = 1:numel(agents)
                relVec = agents(i).position - egoPos;
                dist = norm(relVec);
                if dist > maxRange
                    continue;
                end
                bearing = atan2(relVec(2), relVec(1)) - egoState.yaw;
                bearing = atan2(sin(bearing), cos(bearing));
                if abs(bearing) <= fov / 2
                    inRangeAgents(end + 1) = agents(i); %#ok<AGROW>
                end
            end
        end
    end
end
