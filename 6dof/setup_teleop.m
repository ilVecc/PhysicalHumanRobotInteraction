Ts = 0.001;  % [s]
% master delay
T_m2s = 25 / 1000 / Ts;  % [ 0.025 s]
% slave delay
T_s2m = 25 / 1000 / Ts;  % [ 0.025 s]

%% HUMAN SETUP
% trajectory waypoints
td1 = [              0              1                                  10             18 ];
Xd1 = [ conf_m.init.x0 conf_m.init.x0  conf_m.init.x0+[0.1;0.0;0.0;0;0;0] conf_m.init.x0 ];
td2 = [              0              1                                  10             18 ];
Xd2 = [ conf_m.init.x0 conf_m.init.x0 conf_m.init.x0+[-0.1;0.1;0.1;0;0;0] conf_m.init.x0 ];
td3 = [              0              1                                  10                                 13              18 ];
Xd3 = [ conf_m.init.x0 conf_m.init.x0  conf_m.init.x0+[0.1;0.0;0.0;0;0;0] conf_m.init.x0+[0.1;0.0;0.0;0;0;0]  conf_m.init.x0 ];
% choose trajectory
td = td3;
Xd = Xd3;
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
KD_e = zeros(dh.dof_task); KD_e(1,1) = 10;
KE_e = zeros(dh.dof_task); KE_e(1,1) = 100;
% environment position
Xe = conf_s.init.x0;
Xe(1) = -0.08;

%% observations on the the observer's parameters
%
% |M^-1 * Fh| < f
% 0 < p < 1
% alphas > f;
% lambdas > sqrt(2 ./ (alphas - f)) .* (alphas + f) .* (1 + p) ./ (1 - p);
%
% Simplifing from here, we can write
% alphas = a * f;                                a > 1
% lambdas = b * (a+1)./sqrt(a-1) .* sqrt(f);     b > sqrt(2) = 1.415...
%
% Skrew all of this, just perform an parameter-space optimization wasting
% tons of hours... 
%

%% EQUIVALENT OUTPUT INJECTION SIGNAL FILTER
fc = 100;  % [Hz]
wc = 2*pi*fc;
zdq_B = [0 0 0 0 wc^4];
zdq_A = [1 4*wc 6*wc^2 4*wc^3 wc^4];
clear fc wc

%% MASTER SETUP
% master observer
alphas_m = 75.0 * ones(1,dh.dof_task);  % 74.0
lambdas_m = 4.0 * ones(1,dh.dof_task);  % 18.5
% master impedance controller
M_m = diag(ones(1,dh.dof_task)) * 1;
B_m = diag(ones(1,dh.dof_task)) * 100;
K_m = diag(ones(1,dh.dof_task)) * 0;
% % master command filter
% Fm_fc = 0.01;  % [Hz]
% wc = 2*pi*Fm_fc;
% Fm_B = [0 0 wc^2];
% Fm_A = [1 2*wc wc^2];
% clear Fm_fc wc
bl_obs_m = 1e-4;

%% SLAVE SETUP
% slave observer
alphas_s = alphas_m; 
lambdas_s = lambdas_m;
% slave sliding-mode controller
M_s = diag(ones(1,dh.dof_task)) * 1;
B_s = diag(ones(1,dh.dof_task)) * 15;
K_s = diag(ones(1,dh.dof_task)) * 75;
k_g = 40; % 2*sqrt(dh.dof_task)*(max(alphas_s) + max(alphas_m));
K_g = eye(dh.dof_task) * k_g;
clear k_g
% % slave command filter
% Fs_fc = 30;  % [Hz]
% wc = 2*pi*Fs_fc;
% Fs_B = [0 0 wc^2];
% Fs_A = [1 2*wc wc^2];
% clear Fs_fc wc
bl_obs_s = bl_obs_m;
bl_s = 1e-1;
