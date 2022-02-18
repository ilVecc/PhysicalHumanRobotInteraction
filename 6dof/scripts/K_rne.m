function X = K_rne(dh, q)
    
    T = compute_transformation_matrix(0, dh.dof, dh, q);
    p = T(1:3,4);
    R = T(1:3,1:3);
    
    % Euler angles ZYZ
    eulZYZ = rotm2eul(R, 'ZYZ');  % [phi, theta, psi]
    % phi = atan2(R(2,3), R(1,3));
    % theta = atan2(sqrt(1-R(3,3)^2), R(3,3));
    % psi = atan2(R(3,2), -R(3,1));
    
    X = [p; eulZYZ'];
    
end

