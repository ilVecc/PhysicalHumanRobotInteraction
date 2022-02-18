%% Function that determines if manipulator is in a singular position
%  alert = 0 -> no singularity;
%  alert = 1 -> wrist singularity;
%  alert = 2 -> elbow singularity;
%  alert = 3 -> shoulder singularity.

function alert = singularity_alert(dh, q, epsilon)
    
    if (nargin < 3)
        epsilon = 1e-20;
    end
    
    J = J_rne(dh, q);
    detJ = det(J);
    
    if (abs(detJ) < epsilon) % det => 0
        
        if (q(5) == 0 || q(5) == pi) % wrist singularity -> q(4) and q(5) are parallel
            alert = 1;
        
        elseif (q(3) == 0) % elbow singularity -> planar
            alert = 2;
        
        else               % shoulder singularity -> q(1) and q(2) are
                           % on the same plane as q(5) and q(6)
            alert = 3;
        end
    else
        alert = 0;
    end
    
end