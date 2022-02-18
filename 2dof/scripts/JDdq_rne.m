function JDdq = JDdq_rne(dh, q, dq)
    l = dh.l;
    q1 = q(1);
    q2 = q(2);
    dq1 = dq(1);
    dq2 = dq(2);
    JD = [- l*cos(q1)*dq1 - l*cos(q1+q2)*(dq1+dq2), -l*cos(q1+q2)*(dq1+dq2);
          - l*sin(q1)*dq1 - l*sin(q1+q2)*(dq1+dq2), -l*sin(q1+q2)*(dq1+dq2)];
    JDdq = JD * dq;
end