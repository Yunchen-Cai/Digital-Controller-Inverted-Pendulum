% --- lqr_controller.m ---
clear L K Rn Qn G_noise_map P_observer % Clear previous variables just in case
load('system_model.mat', 'Ad', 'Bd', 'Cd', 'Ts');

% LQR Design
% Q_lqr = diag([1, 10, 1, 10]); % Old weights
Q_lqr = diag([10, 10, 1, 10]); % **** New weights: Increased weight for x ****
R_lqr = 1;
try
    K = dlqr(Ad, Bd, Q_lqr, R_lqr);
catch ME_dlqr
    disp('Error during dlqr calculation. Check matrix dimensions and controllability.');
    rethrow(ME_dlqr);
end
disp('LQR Gain K calculated.');

% Kalman Filter Design - Using DARE
Qn = diag([0.01, 0.01, 0.01, 0.01]); % Process noise covariance
Rn = diag([0.001, 0.001]);           % Measurement noise covariance
G_noise_map = eye(4);                % Assume noise affects states directly

% Solve Discrete Algebraic Riccati Equation for Kalman filter error covariance P
% The observer DARE is: P = Ad*P*Ad' - (Ad*P*Cd')*inv(Cd*P*Cd' + Rn)*(Cd*P*Ad') + G*Qn*G'
% We map this to MATLAB's dare(A, B, Q, R) which solves:
% X = A'XA - (A'XB)*inv(R+B'XB)*(B'XA) + Q
% Mapping: A -> Ad', B -> Cd', Q -> G_noise_map*Qn*G_noise_map', R -> Rn
A_dare = Ad';
B_dare = Cd';
Q_dare = G_noise_map * Qn * G_noise_map'; % Should be 4x4
R_dare = Rn; % Should be 2x2

disp('Solving DARE for Kalman filter...');
try
    % Use the standard call assuming S=0, E=I
    [P_observer, ~, ~] = dare(A_dare, B_dare, Q_dare, R_dare);
    disp('DARE solved successfully.');
catch ME_dare
    disp('Error using dare function. Ensure Control System Toolbox is available.');
    fprintf('Error message: %s\n', ME_dare.message);
    rethrow(ME_dare);
end

% Calculate Kalman Gain L using the steady-state covariance P_observer
% Formula: L = P * C' * inv(C * P * C' + R)
disp('Calculating Kalman gain L from P_observer...');
try
    L = P_observer * Cd' * inv(Cd * P_observer * Cd' + Rn);
    disp('Kalman gain L calculated.');
catch ME_L
    disp('Error calculating L from P_observer. Check matrix dimensions.');
    fprintf('Size of P_observer: %d x %d\n', size(P_observer,1), size(P_observer,2));
    fprintf('Size of Cd'': %d x %d\n', size(Cd',1), size(Cd',2));
    fprintf('Size of Rn: %d x %d\n', size(Rn,1), size(Rn,2));
    rethrow(ME_L);
end

% Save gains
save('controller_gains.mat', 'K', 'L', 'Rn');
disp('LQR and Kalman gains (using DARE) saved to controller_gains.mat');

% Check the size of L immediately
fprintf('Calculated size of L using DARE: %d x %d\n', size(L, 1), size(L, 2));
if ~isequal(size(L), [4, 2])
    error('Calculated Kalman gain L (DARE) does not have the expected 4x2 dimensions!');
end