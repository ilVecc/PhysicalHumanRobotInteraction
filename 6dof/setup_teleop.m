Ts = 0.001;
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
Xd = [conf.init.x0 conf.init.x0+[0.1;0;0;0;0;0] conf.init.x0];
td = [ 0, 8, 16 ];

%% ENVIRONMENT SETUP
KD_e = zeros(dh.dof_task);
KD_e(1,1) = 10;
KE_e = zeros(dh.dof_task);
KE_e(1,1) = 100;
Xe = conf.init.x0;
Xe(1) = -0.08;

%% considerations on the the observer's parameters

%
% SLIDING-MODE OBSERVER
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
fc = 5;  % [Hz]
wc = 2*pi*fc;
zdq_B = [0 0 0 wc^3];
zdq_A = [1 3*wc 3*wc^2 wc^3];
clear fc wc

%% MASTER SETUP
% master observer
alphas_m = 75.0 * ones(1,dh.dof_task);  % 74.0
lambdas_m = 4.0 * ones(1,dh.dof_task);  % 18.5
% master impedance controller
M_m = diag(ones(1,dh.dof_task)) * 1;
B_m = diag(ones(1,dh.dof_task)) * 100;
K_m = diag(ones(1,dh.dof_task)) * 0;
% master command filter
% Fm_fc = 30;  % [Hz]
% wc = 2*pi*Fm_fc;
% Fm_B = [0 0 wc^2];
% Fm_A = [1 2*wc wc^2];


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
% slave command filter
% Fs_fc = 30;  % [Hz]
% wc = 2*pi*Fs_fc;
% Fs_B = [0 0 wc^2];
% Fs_A = [1 2*wc wc^2];
