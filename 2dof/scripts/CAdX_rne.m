function CAdX = CAdX_rne(dh, q, dq)
    BA = BA_rne(dh,q);
    JA = JA_rne(dh,q);
    B = B_rne(dh,q);
    C = C_rne(dh,q,dq);
    JDdq = JDdq_rne(dh,q,dq);
    TA = TA_rne(dh,q);
    TAD = TAD_rne(dh,q,dq);

    CAdX = BA*(JA/B*C*dq - TA\(JDdq - TAD*JA*dq));
end