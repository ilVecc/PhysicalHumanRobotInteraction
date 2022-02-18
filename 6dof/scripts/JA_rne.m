%%
% Analytical Jacobian
% dh: dh table of the robot
% q: joints position

function JA = JA_rne(dh, q)
    TA = TA_rne(dh, q);
    rpinv = (TA'*TA) \ TA';
    JA = rpinv * J_rne(dh, q);
end

