%% UAV Swarm Simulation Results Visualization
% This script allows visualizing the results of a previously run simulation

% Clear any existing figures
close all;

% Check if variables exist in workspace, if not try to load them
if ~exist('uavBodies', 'var') || ~exist('uavBrains', 'var') || ~exist('cloud', 'var')
    try
        if exist('simulation_results.mat', 'file')
            fprintf('Loading simulation results...\n');
            load('simulation_results.mat');
            fprintf('Results loaded successfully.\n');
        else
            error('No simulation results found. Please run run_simulation.m first.');
        end
    catch e
        fprintf('Error loading simulation results: %s\n', e.message);
        return;
    end
end

% Verify that we have all required data
if ~exist('uavBodies', 'var') || ~exist('uavBrains', 'var') || ~exist('cloud', 'var')
    error('Required simulation data is missing. Please run run_simulation.m again.');
end

% Extract data for visualization
uavCount = length(uavBodies);
operational = zeros(1, uavCount);
battery = zeros(1, uavCount);
positions = zeros(uavCount, 2);
states = cell(1, uavCount);

% State names for display
stateNames = {'Inactive', 'Start', 'Calibration', 'Search', 'Approach', 'Track', 'Land'};

% Collect UAV data
for i = 1:uavCount
    operational(i) = uavBodies(i).operational;
    battery(i) = uavBodies(i).batteryLife;
    positions(i,:) = uavBodies(i).getGpsPos();
    states{i} = stateNames{uavBrains(i).currentState + 1}; % +1 because enum starts at 0
end

%% Create visualization
figure('Name', 'UAV Swarm Simulation Results', 'Position', [100, 100, 1200, 800]);

% 1. Plot UAV positions and their operational status
subplot(2, 2, 1);
hold on;
% Plot map boundaries
plot([min(cloud.x), max(cloud.x), max(cloud.x), min(cloud.x), min(cloud.x)], ...
     [min(cloud.y), min(cloud.y), max(cloud.y), max(cloud.y), min(cloud.y)], 'k--');
% Plot operational UAVs
plot(positions(operational==1, 1), positions(operational==1, 2), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
% Plot non-operational UAVs
plot(positions(operational==0, 1), positions(operational==0, 2), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
hold off;
title('UAV Positions');
xlabel('X Position (m)');
ylabel('Y Position (m)');
axis equal;
grid on;
legend('Map Boundary', 'Operational', 'Non-operational', 'Location', 'best');

% 2. Battery life histogram
subplot(2, 2, 2);
histogram(battery, 10);
title('UAV Battery Life Distribution');
xlabel('Remaining Battery Life (seconds)');
ylabel('Number of UAVs');
grid on;

% 3. UAV States pie chart
subplot(2, 2, 3);
stateCounts = zeros(1, 7); % 7 possible states
for i = 1:7
    stateCounts(i) = sum(cell2mat(cellfun(@(x) strcmp(x, stateNames{i}), states, 'UniformOutput', false)));
end
pie(stateCounts, stateNames);
title('UAV States');

% 4. Operational vs. Non-operational
subplot(2, 2, 4);
operationalCount = sum(operational);
nonOperationalCount = uavCount - operationalCount;
bar([operationalCount, nonOperationalCount]);
title('Operational Status');
xticklabels({'Operational', 'Non-operational'});
ylabel('Number of UAVs');
grid on;
ylim([0, uavCount]);

% Add text with percentages
text(1, operationalCount/2, [num2str(round(100*operationalCount/uavCount)) '%'], ...
    'HorizontalAlignment', 'center', 'Color', 'w', 'FontWeight', 'bold');
text(2, nonOperationalCount/2, [num2str(round(100*nonOperationalCount/uavCount)) '%'], ...
    'HorizontalAlignment', 'center', 'Color', 'w', 'FontWeight', 'bold');

% Overall title
sgtitle('UAV Swarm Simulation Analysis', 'FontSize', 14, 'FontWeight', 'bold');

% Print summary statistics
fprintf('\n--- Simulation Summary ---\n');
fprintf('Total UAVs: %d\n', uavCount);
fprintf('Operational UAVs: %d (%.1f%%)\n', operationalCount, 100*operationalCount/uavCount);
fprintf('Non-operational UAVs: %d (%.1f%%)\n', nonOperationalCount, 100*nonOperationalCount/uavCount);
fprintf('Average remaining battery: %.1f seconds\n', mean(battery));
fprintf('Minimum remaining battery: %.1f seconds\n', min(battery));
fprintf('\nUAV States:\n');
for i = 1:7
    if stateCounts(i) > 0
        fprintf('  %s: %d UAVs\n', stateNames{i}, stateCounts(i));
    end
end 