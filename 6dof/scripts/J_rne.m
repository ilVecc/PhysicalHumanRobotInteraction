%%
% Geometric Jacobian
% dh: dh table of the robot 
% q: joints position

function J = J_rne(dh, q, ee, index)

    if nargin == 4
        T = compute_transformation_matrix(0, index, dh, q);
        ee = [ee' 1]';
        pe = T * ee;
        pe = pe(1:3,:);
        n = index;
    else
        T = compute_transformation_matrix(0, length(q), dh, q);
        pe = T(1:3, 4);
        n = length(q);
    end

    J = zeros(length(q));
    zi_m_1 = [0 0 1]';
    pi_m_1 = [0 0 0]';
    
    J(1:3, 1) = cross(zi_m_1, (pe-pi_m_1));
    J(4:6, 1) = zi_m_1;
   
    for i=2:n
        Ti_m_1 = compute_transformation_matrix(0, i-1, dh, q);
        
        zi_m_1 = Ti_m_1(1:3, 3);
        pi_m_1 = Ti_m_1(1:3,4);
        
        J(1:3, i) = cross(zi_m_1, (pe - pi_m_1));
        J(4:6, i) = zi_m_1;
    end

end

