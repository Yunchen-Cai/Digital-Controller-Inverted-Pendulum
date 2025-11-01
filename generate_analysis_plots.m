% --- MATLAB Code to Generate Analysis Plots for Section 6 ---
clear all; close all; clc;

%% --- 1. System Definition and Discretization ---
% (Same as before - defines Ad, Bd, Cd, Dd, Ts, sysd_ss, G_theta)
m_c = 1.1; m_p = 0.1; L = 0.5; g = 9.81;
A = [0 0 1 0; 0 0 0 1; 0 (m_p*g)/m_c 0 0; 0 ((m_c+m_p)*g)/(m_c*L) 0 0];
B = [0; 0; 1/m_c; 1/(m_c*L)];
C = [1 0 0 0; 0 1 0 0]; D = [0; 0];
sys = ss(A, B, C, D); Ts = 0.01;
sysd = c2d(sys, Ts, 'zoh');
Ad = sysd.A; Bd = sysd.B; Cd = sysd.C; Dd = sysd.D;
sysd_ss = ss(Ad, Bd, Cd, Dd, Ts); % Discrete state-space object
G_theta = tf(sysd_ss(2, 1)); % TF from u to theta

disp('System model defined.');

%% --- 2. Define FINAL Controller Parameters ---

% LQR Controller (Final Adjusted Tuning: Q=diag([10,10,1,10]))
if exist('controller_gains.mat', 'file')
    load('controller_gains.mat', 'K', 'L', 'Rn');
    disp('Loaded FINAL K, L, Rn from controller_gains.mat');
    if ~isequal(size(L), [4, 2])
        error('Loaded Kalman gain L does not have 4x2 dimensions! Re-run lqr_controller.m (DARE version).');
    end
else
    error('controller_gains.mat not found. Please run lqr_controller.m (DARE version) first.');
end

% PID Controller (Final Tuned Filtered PD-PD parameters)
Kp_theta = -20;
Ki_theta = 0;
Kd_theta = -2;
% IMPORTANT NOTE for frequency analysis:
% Modeling the filter and the cascade structure accurately for Bode/RLocus
% is complex. We will plot the simplified INNER LOOP PD characteristics.
z = tf('z', Ts);
Deriv_BE = (z-1)/(Ts*z); % Backward Euler approx for D-term TF
C_pd_inner_tf = Kp_theta + Kd_theta * Deriv_BE; % Unfiltered PD TF for analysis
C_pd_inner_tf.Variable = 'z';

disp('Final controller parameters defined/loaded.');

%% --- 3. Generate Analysis Plots for Section 6 ---

% --- Figure X: Root Locus of Inner Loop Plant (G_theta) ---
% Note: This only shows the plant characteristics, not the compensated system poles.
% It's useful context but doesn't directly compare controllers.
figure('Name', 'Plant Root Locus'); % Use descriptive name
rlocus(G_theta);
hold on;
theta_circle = linspace(0, 2*pi, 200);
plot(cos(theta_circle), sin(theta_circle), 'k:'); % Unit circle
hold off;
title('Root Locus of Inner Loop Plant (G_{\theta}(z)) - For Context');
grid on;
saveas(gcf, 'Figure_Comp_Plant_RLocus.png'); % Suggests context plot
disp('Plant Root Locus plot generated and saved.');

% --- Figure Y: Bode Plot of PID Compensated INNER Loop ---
figure('Name', 'PID Inner Loop Bode');
Loop_pd_inner = C_pd_inner_tf * G_theta;
bode(Loop_pd_inner); % Create Bode plot
grid on;

% --- CHANGE THE TITLE COMMAND AGAIN ---
% Original line causing error:
% title('Bode Plot of PID Compensated Inner Loop', ...
%       '(Simplified: PD only, No Filter, No Outer Loop)');

% New version using sprintf to create a single multi-line string:
title_str = sprintf('Bode Plot of PID Compensated Inner Loop\n(Simplified: PD only, No Filter, No Outer Loop)');
title(title_str);
% --- END CHANGE ---

margin(Loop_pd_inner); % Show margins on the plot
[Gm_pid, Pm_pid] = margin(Loop_pd_inner); % Get margin values
fprintf('Simplified PID Inner Loop: PM = %.1f deg, GM = %.1f dB\n', Pm_pid, 20*log10(Gm_pid));
saveas(gcf, 'Figure_Comp_PID_Bode.png');
disp('Simplified PID Inner Loop Bode plot generated and saved.');

% --- Figure Z: Nyquist Plot of LQR Compensated System ---
figure('Name', 'LQR Nyquist');
% Create state-space controller object (Observer + Gain K)
Ac = Ad - L*Cd - Bd*K; Bc = L; Cc = -K; Dc = 0;
ctrl_lqr_ss = ss(Ac, Bc, Cc, Dc, Ts);
% Define plant with analysis points
plant_ap = ss(Ad, Bd, Cd, Dd, Ts);
plant_ap.InputName = 'u_plant'; plant_ap.OutputName = 'y';
ctrl_lqr_ss.InputName = 'y'; ctrl_lqr_ss.OutputName = 'u_plant';
% Calculate loop transfer function at plant input
try
    Lsens = loopsens(plant_ap, ctrl_lqr_ss);
    LoopGain_LQR_Input = Lsens.Li;
    nyquist(LoopGain_LQR_Input);
    axis equal; xlim([-2 2]); ylim([-2 2]); % Adjust limits if needed
catch ME_loopsens
     warning('loopsens failed. Plotting regulator loop gain -K(zI-Ad)^{-1}Bd instead.');
     RegLoopGain = K * tf(ss(Ad,Bd,eye(4),0*Bd,Ts));
     nyquist(-RegLoopGain); axis equal; xlim([-2 2]); ylim([-2 2]);
end
grid on;
title('Nyquist Plot of Final LQR Compensated System (Loop at Plant Input)');
saveas(gcf, 'Figure_Comp_LQR_Nyquist.png');
disp('LQR Nyquist plot generated and saved.');

% --- Figure W: Bode Diagram of LQR Compensated System ---
figure('Name', 'LQR Bode');
try
    bode(LoopGain_LQR_Input); % Use loop gain from Nyquist calculation
    grid on;
    [Gm_lqr, Pm_lqr] = margin(LoopGain_LQR_Input); % Get & display margins
    fprintf('LQR System Loop Gain: PM = %.1f deg, GM = %.1f dB\n', Pm_lqr, 20*log10(Gm_lqr));
catch ME_bode_lqr
    warning('Bode plot using loopsens failed. Plotting regulator loop gain instead.');
    RegLoopGain = K * tf(ss(Ad,Bd,eye(4),0*Bd,Ts));
    bode(-RegLoopGain); grid on;
    [Gm_lqr_alt, Pm_lqr_alt] = margin(-RegLoopGain);
    fprintf('LQR Regulator Loop Gain: PM = %.1f deg, GM = %.1f dB\n', Pm_lqr_alt, 20*log10(Gm_lqr_alt));
end
title('Bode Diagram of Final LQR Compensated System (Loop at Plant Input)');
saveas(gcf, 'Figure_Comp_LQR_Bode.png');
disp('LQR Bode plot generated and saved.');

disp('All comparison analysis plots generated and saved.');