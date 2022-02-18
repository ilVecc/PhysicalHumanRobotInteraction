function C = C_rne(dh, q, dq)
    th2 = dh.m2*dh.l^2;
    C = [-2*th2*dq(2)*sin(q(2)), -th2*dq(2)*sin(q(2));
            th2*dq(1)*sin(q(2)),                    0];
end