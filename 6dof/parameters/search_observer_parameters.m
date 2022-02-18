%% setup
Ts = 0.001;

% get best alphas and lambdas
range_lambdas = 0.5:0.5:60;
range_alphas = 60.5:0.5:90;
[lambdas, alphas] = ndgrid(range_lambdas, range_alphas);
alphas = reshape(alphas, [], numel(alphas));
lambdas = reshape(lambdas, [], numel(lambdas));
% EQUIVALENT OUTPUT INJECTION SIGNAL FILTER
fc = 5;  % [Hz]
wc = 2*pi*fc;
zdq_B = [0 0 0 wc^3];
zdq_A = [1 3*wc 3*wc^2 wc^3];

for i=1:numel(alphas)
    in = Simulink.SimulationInput('InverseDynamicsTest');
    in = in.setVariable('alphas_m', alphas(i) * ones(1,dh.dof));
    in = in.setVariable('lambdas_m', lambdas(i) * ones(1,dh.dof));
    ins(i) = in;
end
clear i in fc n wc alphas lambdas;

%% launch simulation
outs = parsim(ins, 'UseFastRestart','on', 'ShowSimulationManager','on', 'StopOnError','off', 'TransferBaseWorkspaceVariables','on');

%% reshape data
outs = reshape(outs, length(range_lambdas), length(range_alphas));
%outs = reshape(outs, [], 14400);

%% shrink data
for i=1:size(outs, 1)
    for j=1:size(outs, 2)
        results(i, j).success = strcmp(outs(i, j).ErrorMessage, '');
        if results(i, j).success
            results(i, j).time = outs(i, j).tout;
            results(i, j).vars = {'error_X', 'error_q', 'error_qd', 'error_h'};
            results(i, j).data = cat(3, ...
                squeeze(outs(i, j).error_X.Data), ...
                squeeze(outs(i, j).error_q.Data)', ...
                squeeze(outs(i, j).error_dq.Data)', ...
                squeeze(outs(i, j).error_h.Data));
        end
    end
end
clear i j;


%% collect data
for i=1:size(results, 1)
    for j=1:size(results, 2)
        if results(i, j).success
            error(i, j) = 1/mean(abs(results(i, j).data(:, :, 3)), 'all');
        else
            error(i, j) = 0;
        end
    end
end
clear i j;


%% find best
[~, i] = max(error, [], 'all');
[i, j] = ind2sub([length(range_lambdas) length(range_alphas)], i);
best_lambda = range_lambdas(i);
best_alpha = range_alphas(j);
clear i j;


%% show heatmap
heatmap(error, ...
    'XLabel','alphas', 'XDisplayLabels',range_alphas, ...
    'YLabel','lambdas', 'YDisplayLabels',range_lambdas, ...
    'GridVisible', 'off');
