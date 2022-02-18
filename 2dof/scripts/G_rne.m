function G = G_rne(params, q, gravity)
    l = params.l;
    m1 = params.m1;
    m2 = params.m2;
    q1 = q(1);
    g = 9.81;
    G = [(m1/2 + m2)*l*cos(q1);
                             0] * g;
end
