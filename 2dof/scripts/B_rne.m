function B = B_rne(dh, q)
    th1 = dh.m1*dh.l^2;
    th2 = dh.m2*dh.l^2;
    q2 = q(2);
    B = [th1 + 2*th2*(1 + cos(q2)), th2*(1 + cos(q2));
                 th2*(1 + cos(q2)),               th2];
end
