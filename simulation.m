% --- simulation.m ---
clear all; close all; clc;
% Load model and controller gains
run('model.m');
% Choose controller type: 'PID' or 'LQR'
controller_type = 'PID'; % Change to 'PID' to run PID simulation

if strcmp(controller_type, 'LQR')
    run('lqr_controller.m');
    load('controller_gains.mat', 'K', 'L', 'Rn');
else
    % Define PID gains here or load from a file
    Kp_theta = 20; Ki_theta = 0; Kd_theta = 2; % Your current gains
    Kp_x = -0.5; Kd_x = -0.5; % Keep outer loop disabled for now
    % Define Rn for noise consistency if needed for PID sim
    Rn_pid = diag([0.001, 0.001]); % Example
    Rn = Rn_pid; % Use a common variable name
end

% Simulation Parameters
T_final = 10; % Simulation time
% Ensure Ts is loaded or defined
if ~exist('Ts', 'var')
    load('system_model.mat', 'Ts');
end
t = 0:Ts:T_final;
N = length(t);

% Initial conditions
z_true = [0; 0.1; 0; 0]; % True initial state [x, theta, x_dot, theta_dot]
if strcmp(controller_type, 'LQR')
    hat_z = z_true; % Initial state estimate (can start differently)
end

% Data logging
Z_log = zeros(4, N);
U_log = zeros(1, N);
Y_log = zeros(2, N);
if strcmp(controller_type, 'LQR')
    HAT_Z_log = zeros(4, N);
end

% PID specific initializations
if strcmp(controller_type, 'PID')
    e_theta_prev = 0;
    e_x_prev = 0;
    e_theta_int = 0;
    % **** START FILTER INITIALIZATION ****
    theta_dot_filtered_prev = 0; % Initialize filtered velocity estimate
    alpha = 0.8;                 % Filter constant (tune this: 0 < alpha < 1)
    % **** END FILTER INITIALIZATION ****
end

% Simulation Loop
for k = 1:N
    % --- Measurement ---
    y = Cd * z_true; % Ideal measurement
    % Add noise
    y = y + sqrt(diag(Rn)).*randn(2,1); % Using Rn from LQR for consistency
    % Apply sensor limits
    y(1) = min(max(y(1), -0.25), 0.25);
    y(2) = min(max(y(2), -0.21), 0.21);

    % --- Control Calculation ---
    if strcmp(controller_type, 'LQR')
        % ... (LQR logic remains the same) ...
        u = -K * hat_z;
        u = min(max(u, -10), 10); % Saturation
    else % PID
        % Error calculation
        e_theta = 0 - y(2);
        e_x = 0 - y(1);

        % **** START FILTERED DERIVATIVE CALCULATION ****
        % 1. Estimate current angular velocity using finite difference
        theta_dot_est = (e_theta - e_theta_prev) / Ts;

        % 2. Apply low-pass filter
        theta_dot_filtered = alpha * theta_dot_filtered_prev + (1 - alpha) * theta_dot_est;

        % 3. Calculate D-term using filtered estimate
        D_theta = Kd_theta * theta_dot_filtered;
        % **** END FILTERED DERIVATIVE CALCULATION ****

        % PI terms for theta (I_theta is currently 0)
        P_theta = Kp_theta * e_theta;
        I_theta = Ki_theta * e_theta_int; % Still 0 if Ki_theta = 0
        u_theta = P_theta + I_theta + D_theta;

        % PD terms for x (currently disabled)
        % Note: If/when you enable Kd_x, you might want to filter x_dot_est similarly
        x_dot_est = (e_x - e_x_prev) / Ts; % Example unfiltered estimate for x_dot
        P_x = Kp_x * e_x;
        D_x = Kd_x * x_dot_est; % Using unfiltered for now
        u_x = P_x + D_x;

        % Total control input
        u = u_theta + u_x;

        % Saturation
        u_saturated = min(max(u, -10), 10);

        % Anti-windup
        if abs(u) < 10
            e_theta_int = e_theta_int + e_theta * Ts;
        end

        % Update previous values for next step
        e_theta_prev = e_theta;
        e_x_prev = e_x;
        % **** UPDATE FILTER STATE ****
        theta_dot_filtered_prev = theta_dot_filtered; % Update for next iteration
        % **** END UPDATE ****
        u = u_saturated;
    end

    % --- Log data ---
    Z_log(:, k) = z_true;
    U_log(k) = u;
    Y_log(:, k) = y;
    if strcmp(controller_type, 'LQR')
        HAT_Z_log(:, k) = hat_z;
    end

    % --- System Update ---
    z_true = Ad * z_true + Bd * u;
    % Add process noise if desired: z_true = z_true + sqrt(Qn) * randn(4,1);

    % --- Observer Update (LQR only) ---
    if strcmp(controller_type, 'LQR')
        % ... (LQR observer logic remains the same) ...
        hat_z_predict = Ad * hat_z + Bd * u;
        hat_z = hat_z_predict + L * (y - Cd * hat_z_predict);
    end
end

% --- Plotting ---
% ... (Plotting code remains the same) ...
figure;
subplot(3,1,1); plot(t, Z_log(1,:)); title('Cart Position (x)'); ylabel('m');
subplot(3,1,2); plot(t, Z_log(2,:)); title('Pendulum Angle (theta)'); ylabel('rad');
subplot(3,1,3); plot(t, U_log); title('Control Input (f)'); ylabel('N'); xlabel('Time (s)');

if strcmp(controller_type, 'LQR')
    figure;
    subplot(2,1,1); plot(t, Z_log(3,:), 'b', t, HAT_Z_log(3,:), 'r--'); title('Cart Velocity (x\_dot)'); legend('True', 'Estimated');
    subplot(2,1,2); plot(t, Z_log(4,:), 'b', t, HAT_Z_log(4,:), 'r--'); title('Pendulum Velocity (theta\_dot)'); legend('True', 'Estimated'); xlabel('Time (s)');
end