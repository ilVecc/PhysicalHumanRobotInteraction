Ts = 0.001;  % [s]
% master delay
T_m2s = 25 / 1000 / Ts;  % [ 0.025 s]
% slave delay
T_s2m = 25 / 1000 / Ts;  % [ 0.025 s]

%% HUMAN SETUP
td1 = [            0                    8           16 ];
Xd1 = [ conf.init.x0 conf.init.x0+[0.1;0] conf.init.x0 ];
% choose trajectory
td = td1;
Xd = Xd1;
% trajectory controller
KD_h = eye(dh.dof_task) * 50.0;
KP_h = eye(dh.dof_task) * 350.0;
% human sensing filter
fc = 10;  % [Hz]
wc = 2*pi*fc;
FdXh_B = [0 0 0 0 wc^4];
FdXh_A = [1 4*wc 6*wc^2 4*wc^3 wc^4];
clear fc wc;
fc = 60;  % [Hz]
wc = 2*pi*fc;
FXh_B = [1];
FXh_A = [1];
clear fc wc;

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
% clear Fm_fc wc
bl_obs_m = 1e-4;

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
% clear Fs_fc wc
bl_obs_s = bl_obs_m;
bl_s = 1e-1;
