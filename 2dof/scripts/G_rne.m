function G = G_rne(dh, q, gravity)
    l = dh.l;
    m1 = dh.m1;
    m2 = dh.m2;
    q1 = q(1);
    g = 9.81;
    G = [(m1/2 + m2)*l*cos(q1);
                             0] * g;
end
