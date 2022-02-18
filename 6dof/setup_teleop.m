Ts = 0.001;
n = 6;

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

% SLIDING-MODE OBSERVER
% f = 15 * ones(1,n);
% a = 1.1;
% b = 1.01 * sqrt(2);
% alphas = a .* f;
% lambdas = b .* (a+1)./sqrt(a-1) .* sqrt(f);
% alphas = 36.0 * ones(1,n);
% lambdas = 3.5 * ones(1,n);
% EQUIVALENT OUTPUT INJECTION SIGNAL FILTER
fc = 5;  % [Hz]
wc = 2*pi*fc;
zdq_B = [0 0 0 wc^3];
zdq_A = [1 3*wc 3*wc^2 wc^3];

% master observer
alphas_m = 75.0 * ones(1,n);  % 74.0
lambdas_m = 4.0 * ones(1,n);  % 18.5
% slave observer
%alphas_s = alphas_m; lambdas_s = lambdas_m;
clear f a b wc

%%
G = 15;
% master impedance controller
M_m = diag(ones(1,n)) * 1;
B_m = diag(ones(1,n)) * 5;
K_m = diag(ones(1,n)) * 10;
% slave sliding-mode controller
M_s = diag(ones(1,n)) * 1;
B_s = diag(ones(1,n)) * 4;
K_s = diag(ones(1,n)) * 4;
k_g = 2*sqrt(n)*(max(alphas_s) + max(alphas_m));
k_g = 20;
K_g = eye(n) * k_g;
clear n k_g

% master delay
T_m2s = 0;  % [ms]
% slave delay
T_s2m = 0;  % [ms]

% slave command filter
% Fs_fc = 30;  % [Hz]
% wc = 2*pi*Fs_fc;
% Fs_B = [0 0 wc^2];
% Fs_A = [1 2*wc wc^2];
% master command filter
% Fm_fc = 30;  % [Hz]
% wc = 2*pi*Fm_fc;
% Fm_B = [0 0 wc^2];
% Fm_A = [1 2*wc wc^2];

%% plot
% labels = {'$q_m$', '$\hat{q}_m$'};
% qm = out.logsout.find('qm');
% qm_est = out.logsout.find('qm^');
% figure(1); plot_ts(qm, qm_est, labels);

labels = {'$\dot{q}_m$', '$\dot{\hat{q}}_m$'};
dqm = out.logsout.find('dqm');
dqm_est = out.logsout.find('dqm^');
figure(2); plot_ts(dqm, dqm_est, labels);

% labels = {'$F_h$', '$\hat{F}_h$'};
% Fh = out.logsout.find('Fh');
% Fh_est = out.logsout.find('Fh^');
% figure(3); plot_ts(Fh, Fh_est, labels);

% labels = {'$q_s$', '$\hat{q}_s$'};
% qs = out.logsout.find('qs');
% qs_est = out.logsout.find('qs^');
% figure(4); plot_ts(qs, qs_est, labels);

labels = {'$\dot{q}_s$', '$\dot{\hat{q}}_s$'};
dqs = out.logsout.find('dqs');
dqs_est = out.logsout.find('dqs^');
figure(5); plot_ts(dqs, dqs_est, labels);

% labels = {'$F_e$', '$\hat{F}_e$'};
% Fe = out.logsout.find('Fe');
% Fe_est = out.logsout.find('Fe^');
% figure(6); plot_ts(Fe, Fe_est, labels);

clear qm qm_est dqm dqm_est Fh Fh_est qs qs_est dqs dqs_est Fe Fe_est

%%
function [] = plot_ts(s1, s2, labels)
    t1 = s1.Values.Time;
    d1 = squeeze(s1.Values.Data);
    if size(d1, 1) ~= size(t1, 1)
        d1 = d1';
    end
    t2 = s2.Values.Time;
    d2 = squeeze(s2.Values.Data);
    if size(d2, 1) ~= size(t2, 1)
        d2 = d2';
    end
    for i=1:6
        subplot(2,3,i); plot(t1, d1(:, i)); hold on;
        subplot(2,3,i); plot(t2, d2(:, i)); hold on;
        legend(labels, "Interpreter", "latex");
        hold off;
    end
end