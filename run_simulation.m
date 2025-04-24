%% UAV Swarm Simulation for Pollutant Cloud Tracking
% This script provides an easy interface to run the UAV swarm simulation
% with configurable parameters.

% Clear workspace and command window
clear;
clc;

%% Configure Simulation Parameters
% You can modify these parameters to change the simulation behavior

% Number of UAVs in the swarm
uavCount = 24;

% Time step for simulation (seconds)
dt = 1.0;

% Maximum simulation time (seconds)
maxTime = 1800.0;

% Probability that a UAV will have reduced battery life
uavBatteryFailProb = 0.1;

% Initial spacing between UAVs (meters)
initialUavSpacing = 50.0;

% Minimum distance before UAVs crash (meters)
uavCrashRadius = 15.0;

%% Display Options
% Set which elements to display during the simulation

% Set to false to run the simulation without any visualization (faster)
drawAny = true;

% Draw the pollutant cloud contours
drawCloud = true;

% Draw the UAVs
drawUavs = true;

% Draw UAV AI data (target points, etc.)
drawAI = true;

% Draw pheremones (warning: performance intensive)
drawAIExtra = false;

%% Choose Cloud Data
% Select which cloud dispersion model to use

% Choose cloud data: 1 for cloud1.mat, 2 for cloud2.mat
cloudSelection = 2;

if cloudSelection == 1
    cloudFile = 'cloud1.mat';
else
    cloudFile = 'cloud2.mat';
end

%% Run Simulation
% Save the parameters to a temporary file that sim_start will read
save('simulation_parameters.mat', 'uavCount', 'dt', 'maxTime', 'uavBatteryFailProb', ...
    'initialUavSpacing', 'uavCrashRadius', 'drawAny', 'drawCloud', 'drawUavs', ...
    'drawAI', 'drawAIExtra', 'cloudFile');

% Run the simulation
sim_start 