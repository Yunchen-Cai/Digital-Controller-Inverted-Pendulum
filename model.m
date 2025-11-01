% --- model.m ---
m_c = 1.1; m_p = 0.1; L = 0.5; g = 9.81;

A = [0 0 1 0;
     0 0 0 1;
     0 (m_p*g)/m_c 0 0;
     0 ((m_c+m_p)*g)/(m_c*L) 0 0];

B = [0;
     0;
     1/m_c;
     1/(m_c*L)];

C = [1 0 0 0;
     0 1 0 0];

D = [0; 0];

sys = ss(A, B, C, D);
Ts = 0.01;
sysd = c2d(sys, Ts, 'zoh');

Ad = sysd.A;
Bd = sysd.B;
Cd = sysd.C;
Dd = sysd.D;
save('system_model.mat', 'Ad', 'Bd', 'Cd', 'Dd', 'Ts');