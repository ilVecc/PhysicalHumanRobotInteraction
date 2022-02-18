function TA = TA(X)

    phi = X(4);
    theta = X(5);
    psi = X(6);
    
    % Matrix for ZYZ
    T = [0 -sin(phi) cos(phi)*sin(theta);
         0  cos(phi) sin(phi)*sin(theta);
         1         0          cos(theta)];
    
    TA = [ eye(3)  zeros(3); 
          zeros(3)    T    ];
end
