function BA = BA_rne(dh,q)
    B = B_rne(dh,q);
    JA = JA_rne(dh,q);
    BA = pinv(JA*(B\JA'));
end
