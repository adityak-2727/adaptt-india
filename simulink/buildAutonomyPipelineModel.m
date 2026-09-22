function modelPath = buildAutonomyPipelineModel(scenarioName)
% buildAutonomyPipelineModel - programmatically builds (not hand-authored
% XML/binary) a Simulink model that hosts the REAL closed-loop autonomy
% pipeline (perception -> fusion -> tracking -> prediction (K1) ->
% decision -> planning (K2) -> collision check -> control -> vehicle
% motion) as one MATLAB System block (AutonomyPipelineBlock.m), with 16
% output signals logged to the workspace so every stage's execution and
% the tick-to-tick closed-loop feedback can be directly inspected, plus a
% Stop Simulation block that ends the run automatically once the
% scenario's goal is reached (mirroring main.m's own `break`).
%
% Input:
%   scenarioName - optional, default "villageRoad"; one of the five
%                  existing scenarios/*.m names.
% Output:
%   modelPath - full path to the saved .slx file

if nargin < 1 || isempty(scenarioName)
    scenarioName = "villageRoad";
end

modelName = 'autonomyPipeline';

if bdIsLoaded(modelName)
    close_system(modelName, 0);
end

new_system(modelName);
open_system(modelName);

set_param(modelName, 'StopTime', '60');
set_param(modelName, 'FixedStep', '0.1');
set_param(modelName, 'SolverType', 'Fixed-step');

blockPath = [modelName, '/AutonomyPipeline'];
add_block('simulink/User-Defined Functions/MATLAB System', blockPath);
set_param(blockPath, 'Position', [150, 20, 340, 420]);
set_param(blockPath, 'System', 'AutonomyPipelineBlock');
% This block's callees use struct/cell arrays throughout (tracked
% agents, predicted trajectories, candidates) - not code-generation
% compatible.
set_param(blockPath, 'SimulateUsing', 'Interpreted Execution');

set_param(blockPath, 'ScenarioName', char(scenarioName));

outNames = {'EgoX', 'EgoY', 'EgoYaw', 'EgoVelocity', 'NumTrackedAgents', ...
    'NumPredictedTrajectories', 'DecisionSeverity', 'SelectedCandidateIndex', ...
    'MinTTC', 'IsColliding', 'DistToGoal', 'GoalReached', 'SteeringDeg', ...
    'ThrottleCmd', 'BrakeCmd', 'TickCount'};
workspaceVarNames = {'pipEgoX', 'pipEgoY', 'pipEgoYaw', 'pipEgoVelocity', 'pipNumTracked', ...
    'pipNumPredicted', 'pipDecisionSeverity', 'pipSelectedIndex', ...
    'pipMinTTC', 'pipIsColliding', 'pipDistToGoal', 'pipGoalReached', 'pipSteeringDeg', ...
    'pipThrottleCmd', 'pipBrakeCmd', 'pipTickCount'};

for i = 1:numel(outNames)
    sinkPath = [modelName, '/', outNames{i}];
    add_block('simulink/Sinks/To Workspace', sinkPath);
    yPos = 10 + (i - 1) * 26;
    set_param(sinkPath, 'Position', [420, yPos, 520, yPos + 14]);
    set_param(sinkPath, 'VariableName', workspaceVarNames{i}, 'SaveFormat', 'Array');
    add_line(modelName, ['AutonomyPipeline/', num2str(i)], [outNames{i}, '/1']);
end

% Stop Simulation once the scenario's goal is reached (output port 12,
% GoalReached, 0/1) - mirrors main.m's own `if ... < goalTolerance; break`.
stopPath = [modelName, '/StopWhenGoalReached'];
add_block('simulink/Sinks/Stop Simulation', stopPath);
set_param(stopPath, 'Position', [420, 480, 460, 500]);
add_line(modelName, 'AutonomyPipeline/12', 'StopWhenGoalReached/1');

modelPath = fullfile(fileparts(mfilename('fullpath')), [modelName, '.slx']);
save_system(modelName, modelPath);
close_system(modelName, 0);

fprintf('Built and saved %s (scenario=%s)\n', modelPath, scenarioName);

end
