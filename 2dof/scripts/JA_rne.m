function JA = JA_rne(dh, q)
    TA = TA_rne(dh, q);
    rpinv = (TA'*TA) \ TA';
    JA = rpinv * J_rne(dh, q);
end

