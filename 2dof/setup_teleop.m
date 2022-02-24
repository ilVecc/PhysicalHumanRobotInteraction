Ts = 0.001;  % [s]
% master delay
T_m2s = 25 / 1000 / Ts;  % [ 0.025 s]
% slave delay
T_s2m = 25 / 1000 / Ts;  % [ 0.025 s]

%% HUMAN SETUP
z = 0.0;
wn = 250;
KD_h = eye(dh.dof_task) * 2*z*wn;
KP_h = eye(dh.dof_task) * wn^2;
clear z wn
Xd = [conf.init.x0 conf.init.x0+[0.1;0] conf.init.x0];
td = [ 0, 8, 16 ];

%% ENVIRONMENT SETUP
KD_e = eye(dh.dof_task) * 1000;
KE_e = eye(dh.dof_task) * 10000;
Xe = [0.50 0.43];

%% EQUIVALENT OUTPUT INJECTION SIGNAL FILTERS
fc = 10;  % [Hz]
wc = 2*pi*fc;
zdq_B = [0 0 0 wc^3];
zdq_A = [1 3*wc 3*wc^2 wc^3];
clear fc wc

%% MASTER SETUP
% master observer
alphas_m = 25.0 * ones(1,dh.dof);
lambdas_m = 7.5 * ones(1,dh.dof);
% master impedance controller
M_m = diag(ones(1,dh.dof)) * 22;
B_m = diag(ones(1,dh.dof)) * 32;
K_m = diag(ones(1,dh.dof)) * 22;
% master command filter
% Fm_fc = 30;  % [Hz]
% wc = 2*pi*Fm_fc;
% Fm_B = [0 0 wc^2];
% Fm_A = [1 2*wc wc^2];

%% SLAVE SETUP
% slave observer
alphas_s = 16.5 * ones(1,dh.dof);
lambdas_s = 3.0 * ones(1,dh.dof);
% slave sliding-mode controller
M_s = diag(ones(1,dh.dof)) * 1;
B_s = diag(ones(1,dh.dof)) * 15;
K_s = diag(ones(1,dh.dof)) * 75;
k_g = 40; %k_g = 2*sqrt(n)*(max(alphas_s) + max(alphas_m));
K_g = eye(dh.dof) * k_g;
clear k_g
% slave command filter
% Fs_fc = 30;  % [Hz]
% wc = 2*pi*Fs_fc;
% Fs_B = [0 0 wc^2];
% Fs_A = [1 2*wc wc^2];
