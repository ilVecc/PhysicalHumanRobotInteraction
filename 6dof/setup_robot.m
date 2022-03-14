addpath('scripts');

%% robot parameters
dh.issym = false;
dh.dof = 6;
dh.dof_task = 6;
% DH table
dh.d = [0.089159 0 0 0.10915 0.09465 0.0823];
dh.alpha = [pi/2 0 0 pi/2 -pi/2 0];
dh.a = [0 -0.42500 -0.39225 0 0 0];
% Mass and Centers of mass
dh.m = [3.7000 8.3930 2.2750 1.2190 1.2190 0.1879];
cm1 = [0.0 -0.02561 0.00193];
cm2 = [0.2125 0.0 0.11336];
cm3 = [0.15 0.0 0.0265];
cm4 = [0.0 -0.0018 0.01634];
cm5 = [0.0 0.0018 0.01634];
cm6 = [0.0 0.0 -0.001159];
dh.cm = [cm1' cm2' cm3' cm4' cm5' cm6'];
clear cm1 cm2 cm3 cm4 cm5 cm6;
% Inertial tensor at the centers of mass
i1 = [0.010267 0.010267 0.00666];
i2 = [0.2269 0.2269 0.0151];
i3 = [0.0312168 0.0312168 0.004095];
i4 = [0.002559898976 0.002559898976 0.0021942];
i5 = [0.002559898976 0.002559898976 0.0021942];
i6 = [8.46958911216e-5 8.46958911216e-5 0.0001321171875];
dh.I = zeros(3, 3, 6);
dh.I(:,:,1) = diag(i1);
dh.I(:,:,2) = diag(i2);
dh.I(:,:,3) = diag(i3);
dh.I(:,:,4) = diag(i4);
dh.I(:,:,5) = diag(i5);
dh.I(:,:,6) = diag(i6);
clear i1 i2 i3 i4 i5 i6;
% gravity and friction
gravity = [0 0 9.81]';
%Fv = 0;
%Fs = 0;

% initial configuration
conf_m.init.q0 = [-1.6007 -1.7271 -2.203 0.7628 1.5951 -0.031]';
conf_m.init.dq0 = [0 0 0 0 0 0]';
conf_m.init.x0 = K_rne(dh, conf_m.init.q0);
conf_m.init.dx0 = [0 0 0 0 0 0]';

conf_s.init.q0 = [-1.6007 -1.7271 -2.203 0.7628 1.5951 -0.031]';
conf_s.init.dq0 = [0 0 0 0 0 0]';
conf_s.init.x0 = K_rne(dh, conf_s.init.q0);
conf_s.init.dx0 = [0 0 0 0 0 0]';

% desired configuration
conf.des.qd = [0 0 0 0 0 0]';
conf.des.dqd = [0 0 0 0 0 0]';
conf.des.xd = K_rne(dh, conf.des.qd);
conf.des.dxd = [0 0 0 0 0 0]';
