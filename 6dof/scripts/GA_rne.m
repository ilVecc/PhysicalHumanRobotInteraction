function GA = GA_rne(dh, q, gravity)
    BA = BA_rne(dh,q);
    JA = JA_rne(dh,q);
    B = B_rne(dh,q);
    G = G_rne(dh,q,gravity);

    GA = BA*JA/B*G;
end

