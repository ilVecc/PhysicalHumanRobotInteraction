function J = J_rne(dh, q)
    l = dh.l;
    q1 = q(1);
    q2 = q(2);
    J = [-l*sin(q1) - l*sin(q1+q2), -l*sin(q1+q2);
          l*cos(q1) + l*cos(q1+q2),  l*cos(q1+q2)];
end

