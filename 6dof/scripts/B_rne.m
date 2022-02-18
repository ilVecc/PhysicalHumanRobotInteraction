function B = B_rne(dh, q)
    n = dh.dof;
    B = zeros(n);
    I = eye(n);
    for i = 1:n
        bi = rne_inverse(dh, q, [0 0 0 0 0 0]', I(:,i), [0 0 0]', [0 0 0 0 0 0]');
        B(:,i) = bi;
    end    
end

