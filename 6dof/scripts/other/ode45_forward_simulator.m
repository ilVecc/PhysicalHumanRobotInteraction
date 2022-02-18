%%
% This function computes the forward dynamic of a robot in an analytic way
% (with Runge-Kutta)
% dh: dh table of the robot
% t1: time
% q0: joints initial position
% qd0: joints initial velocities
% tau: desired torque
% he: end-effector forces (1x6)
% gravity: (1x3)
% Returns:
% t: the time vector
% q: joints position
% qd: joints velocity

function [t, q, qd] = forward_dynamics(dh, t1, q0, qd0, tau, he, gravity, Fv, Fs)
    
    q0 = q0';
    qd0 = qd0';
    q0 = [q0(:); qd0(:)];
    
    %% Velocity and position
    [t, y] = ode45(@private_forward_dynamics, [0 t1], q0, [], dh, tau, he, gravity, Fv, Fs);
    
    q = y(:, 1:n)';
    qd = y(:, n+1:2*n)';
    
end

function xd = private_forward_dynamics(t, x, dh, tau, he, gravity, Fv, Fs)
    
    n = dh.length(d);
    q = x(1:n);
    q_dot = x(n+1:2*n);
    %Fs = 0;
    %Fv = 0;
    %B = B_matrix(dh, q);
    %C = C_matrix(B, q, q_dot);
    %g = g_calc(dh, gravity, q);
    %J = Jacobian(dh, q);
    
    %tau_p = C * q_dot + Fv * q_dot + Fs * sign(q_dot) + g + J' * he;
    tau_p = inv_dyn_eval(dh, q, q_dot, he, Fv, Fs, gravity);
    q_ddot = B \ (tau - tau_p);
    
    xd = [x(n+1:2*n); q_ddot];

end