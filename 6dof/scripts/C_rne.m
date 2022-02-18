function C = C_rne(dh, q, dq)
    
    n = dh.dof;
    
    C = zeros(n, n);
    Csq = zeros(n, n);
        
    for i=1:n
       QD = zeros(n,1);
       QD(i) = 1;
       tau = rne_inverse(dh, q, QD, zeros(n,1), [0 0 0]', [0 0 0 0 0 0]');
       Csq(:,i) = Csq(:,i) + tau;
    end
    
    for i=1:n
        for k=i+1:n
            QD = zeros(n,1);
            QD(i) = 1;
            QD(k) = 1;
            tau = rne_inverse(dh, q, QD, zeros(n, 1), [0 0 0]', [0 0 0 0 0 0]');
            C(:,k) = C(:,k) + (tau - Csq(:,k) - Csq(:,i)) * dq(i)/2;
            C(:,i) = C(:,i) + (tau - Csq(:,k) - Csq(:,i)) * dq(k)/2;
        end
    end
    C = C + Csq * diag(dq);

end