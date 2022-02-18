function G = G_rne(dh, q, gravity)
    G = rne_inverse(dh, q, [0 0 0 0 0 0]', [0 0 0 0 0 0]', gravity);
end

