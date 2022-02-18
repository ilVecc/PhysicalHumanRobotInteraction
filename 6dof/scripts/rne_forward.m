function [q, qd, qdd] = rne_forward(dh, q0, qd0, tau, he, gravity, delta_t)

    n = dh.dof;
    
    qdd = private_fwd_rne(dh, q0, qd0, tau, he, gravity);
    M0 = delta_t * qdd;
    
    qdd1 = private_fwd_rne(dh, q0 + delta_t*qd0/2, qd0 + M0/2, tau, he, gravity);
    M1 = delta_t * qdd1;
    
    qdd2 = private_fwd_rne(dh, q0 + delta_t*qd0/2 + delta_t*M0/4, qd0 + M1/2, tau, he, gravity);
    M2 = delta_t * qdd2;
    
    qdd3 = private_fwd_rne(dh, q0 + delta_t*qd0 + delta_t*M1/2, qd0 + M2, tau, he, gravity);
    M3 = delta_t * qdd3;
    
    q = q0 + delta_t*qd0 + (delta_t/6.0)*(M0 + M1 + M2);
    qd = qd0 + (1.0/6.0)*(M0 + 2*M1 + 2*M2 + M3);
    
end

function qdd = private_fwd_rne(dh, q, qd, tau, he, gravity)
    n = dh.dof;
    qdd_prime = zeros(n,1);
    tau_prime = inv_dyn_rne(dh, q, qd, qdd_prime, gravity, he); 
    B = zeros(n);
    I = eye(n);
    for i=1:n
        B(:,i) = inv_dyn_rne(dh, q, zeros(n,1), I(:,i), [0 0 0]', he);
    end
    
    qdd = inv(B) * (tau - tau_prime);   
end
