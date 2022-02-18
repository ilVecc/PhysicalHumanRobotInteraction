function [sys,x0,str,ts] = fwd_dyn_S_matrix(t,x,u,flag,dh,gravity,q0,qd0)
    
    switch flag
    
        %%%%%%%%%%%%%%%%%%
        % Initialization %
        %%%%%%%%%%%%%%%%%%
        case 0
            [sys,x0,str,ts]=mdlInitializeSizes(dh.dof, q0, qd0);
        
        %%%%%%%%%%%%%%%
        % Derivatives %
        %%%%%%%%%%%%%%%
        case 1
            sys=mdlDerivatives(t,x,u,dh,gravity);
        
        %%%%%%%%%%%
        % Outputs %S
        %%%%%%%%%%%
        case 3
            sys=mdlOutputs(t,x,u);
        
        %%%%%%%%%%%%%%%%%%%
        % Unhandled flags %
        %%%%%%%%%%%%%%%%%%%
        case { 2, 4, 9 }
        sys = [];
        
        %%%%%%%%%%%%%%%%%%%%
        % Unexpected flags %
        %%%%%%%%%%%%%%%%%%%%
        otherwise
            DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));
    
    end

%=============================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=============================================================================
function [sys,x0,str,ts]=mdlInitializeSizes(n, q0, qd0)

    sizes = simsizes;
    sizes.NumContStates  = 2*n;
    sizes.NumDiscStates  = 0;
    sizes.NumOutputs     = 2*n;
    sizes.NumInputs      = n;
    sizes.DirFeedthrough = 0;
    sizes.NumSampleTimes = 1;
    
    sys = simsizes(sizes);
    x0  = [q0; qd0];  %starting positions and velocities
    str = [];
    ts  = [0 0];

%=============================================================================
% mdlDerivatives
% Return the derivatives for the continuous states.
%=============================================================================
function sys=mdlDerivatives(t,x,u,dh,gravity)
    % x = [q; qdot]
    q = x(1 : dh.dof);
    qdot = x(dh.dof+1 : 2*dh.dof);
    tau = u;
    
    B = double(B_recursive_NewtonEulero(dh, q));
    C = double(C_recursive_NewtonEulero(dh, q, qdot));
    G = inv_dyn_recursive_NewtonEulero(dh, q, [0 0 0 0 0 0]', [0 0 0 0 0 0]', gravity);
    
    qddot = B \ (tau - C*qdot - G);
    
    sys = [qdot; qddot];
    %sys = A*x + B*u;

%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
function sys=mdlOutputs(t,x,u)

    sys = x;
