% --- Inside simulation loop (e.g., in simulation_pid.m) ---
% Error calculation
e_theta = 0 - y(2); % Target theta is 0
e_x = 0 - y(1);     % Target x is 0

% PID for theta
P_theta = Kp_theta * e_theta;
I_theta = Ki_theta * e_theta_int; % e_theta_int updated with anti-windup
D_theta = Kd_theta * (e_theta - e_theta_prev) / Ts;
u_theta = P_theta + I_theta + D_theta;

% PD for x
P_x = Kp_x * e_x;
D_x = Kd_x * (e_x - e_x_prev) / Ts;
u_x = P_x + D_x;

% Total control input
u = u_theta + u_x;

% Saturation
u_saturated = min(max(u, -10), 10);

% Anti-windup for theta integral term
if abs(u) < 10
    e_theta_int = e_theta_int + e_theta * Ts;
end

% Update previous errors for next step
e_theta_prev = e_theta;
e_x_prev = e_x;

u = u_saturated; % Use saturated value