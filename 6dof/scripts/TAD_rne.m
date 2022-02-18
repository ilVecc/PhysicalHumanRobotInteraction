function TAD = TAD_rne(dh,q,dq)

    X = K_rne(dh, q);
    phi = X(4);
    theta = X(5);
    psi = X(6);

    dX = JA_rne(dh,q) * dq;
    dphi = dX(4);
    dtheta = dX(5);
    dpsi = dX(6);
    
    % Matrix for ZYZ
    T = [0 -cos(phi)*dphi -sin(phi)*sin(theta)*dphi+cos(phi)*cos(theta)*dtheta;
         0 -sin(phi)*dphi -cos(phi)*sin(theta)*dphi+sin(phi)*cos(theta)*dtheta;
         0              0                                   -sin(theta)*dtheta];
    
    TAD = [zeros(3) zeros(3); 
           zeros(3) T];
end

