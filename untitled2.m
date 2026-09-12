% 1. Load reference waypoints
if exist('waypoints.txt', 'file')
    ref_wp = readmatrix('waypoints.txt', 'CommentStyle', '#');
else
    error('waypoints.txt not found in current working directory.');
end

% 2. Retrieve actual vehicle trajectory (from file or workspace)
if exist('driven_waypoints.txt', 'file')
    act_wp = readmatrix('driven_waypoints.txt');
elseif exist('out', 'var') && isprop(out, 'driven_pos')
    act_wp = out.driven_pos.Data;
    writematrix(act_wp, 'driven_waypoints.txt', 'Delimiter', ',');
elseif exist('driven_pos', 'var')
    if isa(driven_pos, 'timeseries')
        act_wp = driven_pos.Data;
    else
        act_wp = driven_pos;
    end
    writematrix(act_wp, 'driven_waypoints.txt', 'Delimiter', ',');
else
    error('No driven position data found. Ensure the Simulink model has finished running.');
end

% Format matrix dimensions if logged as 3D timeseries [1 x 2 x N]
act_wp = squeeze(act_wp);
if size(act_wp, 1) < size(act_wp, 2) && size(act_wp, 1) <= 3
    act_wp = act_wp.';
end

% 3. Plot comparison graph
figure('Name', 'Target vs Driven Path');
plot(ref_wp(:,1), ref_wp(:,2), 'r--', 'LineWidth', 2); hold on;
plot(act_wp(:,1), act_wp(:,2), 'b-', 'LineWidth', 1.5);
grid on; axis equal;
xlabel('X Position [m]'); ylabel('Y Position [m]');
legend('Reference Waypoints', 'Actual Vehicle Trajectory', 'Location', 'best');
title('Target vs. Driven Vehicle Path');