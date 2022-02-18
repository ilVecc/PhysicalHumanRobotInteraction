addpath('scripts');

%% robot parameters
dh.dof = 2;
dh.m1 = 1;
dh.m2 = 1;
dh.l = 1;

gravity = [0 0 9.81]';

% initial configuration
conf.init.q0 = [-0.5 2.5]';
conf.init.dq0 = [0 0]';
conf.init.x0 = K_rne(dh, conf.init.q0);
conf.init.dx0 = JA_rne(dh, conf.init.q0) * conf.init.dq0;

% desired configuration
conf.des.qd = [0 0]';
conf.des.dqd = [0 0]';
conf.des.xd = K_rne(dh, conf.des.qd);
conf.des.dxd = JA_rne(dh, conf.des.qd) * conf.des.dqd;
