%%
% Calculates Transformation matrix 

function  T = compute_transformation_matrix(from, to, dh, qc)
    % Transformation from one joint to another joint
    % 0<=from<N_DOFS
    % 0<to<=N_DOFS
    
    T = eye(4);
    N_DOFS = length(qc);
    
    % Sanity check
    if (from >= N_DOFS) || (from < 0) || (to <= 0) || (to >  N_DOFS)
        return;
    end
    
    for i = from+1 : to
        
        %{
        dh.theta(i) = qc(i);
        ct = cos(dh.theta(i));
        st = sin(dh.theta(i));
        ca = cos(dh.alpha(i));
        sa = sin(dh.alpha(i));
        %}
        
        ct = cos(qc(i));
        st = sin(qc(i));
        ca = cos(dh.alpha(i));
        sa = sin(dh.alpha(i));
        
        T = T * [ ct    -st*ca   st*sa     dh.a(i)*ct ; ...
                  st    ct*ca    -ct*sa    dh.a(i)*st ; ...
                  0     sa       ca        dh.d(i)    ; ...
                  0     0        0         1          ];
    end
    
end
    