
% trajectory settings
ti = 0;
tf = 10;
ts = 0.001;
qi = [0.2 -0.5 0.3 -2 2.5 3]';
qf = [-0.2 -1.9 0.3 1 1.5 -1.2]';
q0 = qi;

% calculate trajectory
samples = (tf - ti) / ts;
TimeValues = linspace(ti, tf, samples);
DimValues = 6;
DataPosition = [];
DataVelocity = [];
for i=1:length(qi)
    [~,q,qD,~,~,~] = trigonometric_harmonic(ts,ti,tf,qi(i),qf(i));
    DataPosition = [DataPosition q];
    DataVelocity = [DataVelocity qD];
end
clear q qD i qd dotqd

% prepare data for Simulink
qd.time = TimeValues;
qd.signals.values = DataPosition;
qd.signals.dimensions = DimValues;
dotqd.time = TimeValues;
dotqd.signals.values = DataVelocity;
dotqd.signals.dimensions = DimValues;



%% actual time law
function [T,q,qD,qDD,qDDD,qDDDD] = trigonometric_harmonic(ts,ti,tf,qi,qf)

samples = (tf - ti) / ts;
T = linspace(ti, tf, samples);
DQ = qf - qi;
DT = tf - ti;

q = DQ/2*(1 - cos(pi*(T-ti)/DT)) + qi;
qD = (pi/DT)*DQ/2*sin(pi*(T-ti)/DT);
qDD = (pi/DT)^2*DQ/2*cos(pi*(T-ti)/DT);
qDDD = -(pi/DT)^3*DQ/2*sin(pi*(T-ti)/DT);
qDDDD = -(pi/DT)^4*DQ/2*cos(pi*(T-ti)/DT);

end

