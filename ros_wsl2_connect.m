% =========================================================================
% AUTOMATED ROS SETUP FOR WSL2 & CARLA <-> SIMULINK
% Run this script ONCE in MATLAB before pressing RUN in Simulink
% =========================================================================

clc;
fprintf('=== Auto-Configuring ROS Connection for WSL2 ===\n\n');

% 1. Shutdown existing ROS node
rosshutdown;

% 2. Auto-detect dynamic WSL2 IP and Host Windows IP
try
    [status_wsl, wsl_out] = system('wsl hostname -I');
    [status_host, host_out] = system('wsl ip route show default');
    
    if status_wsl == 0 && status_host == 0
        % Extract WSL IP (first IP in output string)
        wsl_ip = strtrim(strtok(wsl_out));
        
        % Extract Host IP (gateway IP from 'default via X.X.X.X')
        tokens = regexp(host_out, 'default via (\d+\.\d+\.\d+\.\d+)', 'tokens');
        if ~isempty(tokens)
            host_ip = tokens{1}{1};
        else
            host_ip = '172.19.48.1'; % Fallback
        end
    else
        error('WSL execution returned non-zero code.');
    end
catch
    warning('Auto-detection failed. Using hardcoded fallback IPs.');
    wsl_ip  = '172.19.52.127';
    host_ip = '172.19.48.1';
end

fprintf('[DETECTED] WSL2 ROS Master IP : %s\n', wsl_ip);
fprintf('[DETECTED] MATLAB Host ROS_IP  : %s\n', host_ip);

% 3. Set Environment Variable
setenv('ROS_IP', host_ip);

% 4. Connect to ROS Master
try
    rosinit(wsl_ip);
    fprintf('\n[SUCCESS] Connected to ROS Master!\n');
catch ME
    error('\n[ERROR] Could not connect to ROS Master at %s.\nEnsure roscore is running in WSL2.\nMessage: %s', wsl_ip, ME.message);
end

% 5. Test Live Handshake with CARLA Odometry
odom_topic = '/carla/ego_vehicle/odometry';
fprintf('[INFO] Testing live data handshake on %s...\n', odom_topic);

try
    sub = rossubscriber(odom_topic);
    odom_data = receive(sub, 3); % 3-second timeout
    
    pos = odom_data.Pose.Pose.Position;
    fprintf('[SUCCESS] Handshake verified!\n');
    fprintf('          Ego Position -> X: %.2fm | Y: %.2fm | Z: %.2fm\n', pos.X, pos.Y, pos.Z);
    fprintf('\n=======================================================\n');
    fprintf('  READY! You can now open and RUN your Simulink model.  \n');
    fprintf('=======================================================\n');
catch
    warning('[WARNING] ROS connected, but no data received on %s.', odom_topic);
    fprintf('          Make sure CARLA ROS bridge is active and spawning the vehicle.\n');
end