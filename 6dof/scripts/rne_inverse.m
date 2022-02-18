%%
% This function computes the inverse dynamic in a recursive way (Euler -
% Newton)
% dh: dh matrix of the robot
% a1: joints position
% a2: joints velocity
% a3: joints acceleration
% G: gravity (1x3)
% a5 = external forces applied on the end-effector 
% tau = torques

function tau = rne_inverse(dh, a1, a2, a3, G, a5)

    z0 = [0;0;1];
    fext = zeros(6, 1);
    n = dh.dof;

    % check that robot object has dynamic parameters for each link
    for j=1:n
        if isempty(dh.cm(j)) || isempty(dh.I(:,:,j)) || isempty(dh.m(j))
            error('dynamic parameters (m, r, I) not set in link %d', j);
        end
    end

    np = 1;
    Q = a1;
    Qd = a2;
    Qdd = a3;
    
    %if numcols(a1) ~= n || numcols(Qd) ~= n || numcols(Qdd) ~= n || ...
    %    numrows(Qd) ~= np || numrows(Qdd) ~= np
    %    error('bad data');
    %end
    grav = G;
    if nargin == 6
        fext = a5;
    end
    
    %{
    if dh.issym || any([isa(Q,'sym'), isa(Qd,'sym'), isa(Qdd,'sym')])
        tau = zeros(np,n);
    else
        tau = zeros(np,n);
    end
    %}
    tau = zeros(np, n);
   
    for p=1:np
        %q = Q(p,:).';
        %qd = Qd(p,:).';
        %qdd = Qdd(p,:).';
        q = Q';
        qd = Qd';
        qdd = Qdd';
        
        Fm = zeros(3,n);
        Nm = zeros(3,n);
        pstarm = zeros(3,n);
        
        Rm = cell(n,1);
        
        % rotate base velocity and acceleration into L1 frame
        w = zeros(3,1);
        wd = zeros(3,1);
        vd = grav(:);

    %
    % init some variables, compute the link rotation matrices
    %
        for j=1:n
            
            Tj = compute_transformation_matrix(j-1, j, dh, q);
            
            % revolute joint
            d = dh.d(j);
          
            alpha = dh.alpha(j);
            % O_{j-1} to O_j in {j}, negative inverse of link xform
            pstar = [dh.a(j); d*sin(alpha); d*cos(alpha)];

            pstarm(:,j) = pstar;
            Rm{j} = Tj(1:3, 1:3);

        end

    %
    %  the forward recursion
    %
        for j=1:n

            Rt = Rm{j}.';    % transpose!!
            pstar = pstarm(:,j);
            r = dh.cm(:, j);
           
            %
            % statement order is important here
            %               
            % revolute axis
                       
            wd = Rt*(wd + z0*qdd(j) + ...
                cross(w,z0*qd(j)));
            w = Rt*(w + z0*qd(j));
            %v = cross(w,pstar) + Rt*v;
                      
            vd = cross(wd,pstar) + ...
                cross(w, cross(w,pstar)) +Rt*vd;
            
            %whos
                       
            vhat = cross(wd,r) + ...
                cross(w,cross(w,r)) + vd;
                                  
            F = dh.m(j)*vhat;
            N = dh.I(:,:,j)*wd + cross(w,dh.I(:,:,j)*w);
            Fm(:,j) = F;
            Nm(:,j) = N;
           
           
        end
        
    %
    %  the backward recursion
    %
  
        fext = fext(:);
        f = fext(1:3);      % force/moments on end of arm
        nn = fext(4:6);

        for j=n:-1:1
            pstar = pstarm(:,j);
          
            %
            % order of these statements is important, since both
            % nn and f are functions of previous f.
            %
            if j == n
                R = eye(3,3);
            else
                R = Rm{j+1};
            end
            r = dh.cm(:, j);
           
            nn = R*(nn + cross(R.'*pstar,f)) + ...
                cross(pstar+r,Fm(:,j)) + ...
                Nm(:,j);
            f = R*f + Fm(:,j);
           
            R = Rm{j};
        
            % revolute
            t = nn.'*(R.'*z0);% + ...
                %dh.G(j)^2 * dh.Jm(j)*qdd(j);
                %dh.friction(qd(j));
            tau(p,j) = t;
           
        end
        % this last bit needs work/testing
        R = Rm{1};
        nn = R*(nn);
        f = R*f;
        wbase = [f; nn];
    end
    tau = tau';
   
end

   
    