function cost6 = costfunction6(X_ref_6,x_k_6,u6)
    
% Definition of variables
Ts=0.01;    
g_bar = [1000,2000,2,2,2,260,260,130];
N1 = 10;

% state space model after time delay estimation
    A_6 = [1 Ts 0 0;
        0 2 0 -1;
        1 0 0 0;
        0 1 0 0];
    B_6 = [0; (g_bar(6)*Ts); 0; 0];
    [n, m] = size(B_6);
    
    % the basic idea, here, is prediction of the future state base on the
    % current information and system model 
    % the detail algorithm is presented in Dr. Prof. Daniel Görges's model
    % prediction control lectures, Technical university of kaiserslautern
    
    Phi_6 = zeros(N1*n,n); 
    Gamma_6 = zeros(N1*n,N1*m);
    
    % for joint 1 prediction update
    for r = 1:N1
    
    Phi_6(((r-1)*n)+1:r*n,1:n) = A_6^r;
    
        for c = 1:N1
            if r >= c
            
                Gamma_6(((r-1)*n)+1:r*n,((c-1)*m)+1:c*m) = A_6^(r-c)*B_6;
            
            end
        end
    end
    % predicted model
    X_k_6 = Phi_6*x_k_6 + Gamma_6*u6;
    
    % the basic idea, here, is input optimization that satisfy the minimum cost 
    % the detail algorithm is presented in Dr. Prof. Daniel Görges's model
    % prediction control lectures, Technical university of kaiserslautern
    
    Omega = zeros(N1*n,N1*n);
    Psi = zeros(N1*m,N1*m);
    
    Q = [120 0 0 0;
        0 1 0 0;
        0 0 20 0;
        0 0 0 0.1];
    R = 1;
    
    for r = 1:N1
            
            Omega((r-1)*n+1:r*n,(r-1)*n+1:r*n) = Q;
    
            Psi((r-1)*m+1:r*m,(r-1)*m+1:r*m) = R;
    
    end
    
    % optimization * objective function*
    
        cost6 = (X_k_6-X_ref_6)'*Omega*(X_k_6-X_ref_6) + u6'*Psi*u6;
   
end