%%
% This function computes the vector obtained by deriving the potential
% energy (joint space)
% dh: dh table of the robot
% gravity: (1x3)
% q: joints position


function G = g_q(dh, gravity, q)

    G = zeros(dh.dof, 1);
    for i = 1:dh.dof
        for j = 1:dh.dof
            J = Jacobian(dh, q, dh.cm(1:3, j), j);
            G(i) = G(i) + (dh.m(j) * gravity' * J(1:3, i));
        end
    end

end

