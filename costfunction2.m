function cost2 = costfunction2(X_ref_2,x_k_2,u2)
    
% Definition of variables
Ts=0.01;   
g_bar = [1000,2000,2,2,2,260,260,130];
N1 = 10;

% state space model after time delay estimation
     A_2 = [1 Ts 0 0;
        0 2 0 -1;
        1 0 0 0;
        0 1 0 0];
    B_2 = [0; (g_bar(2)*Ts); 0; 0];
    [n, m] = size(B_2);
    
    % the basic idea, here, is prediction of the future state base on the
    % current information and system model 
    % the detail algorithm is presented in Dr. Prof. Daniel Görges's model
    % prediction control lectures, Technical university of kaiserslautern
    
    Phi_2 = zeros(N1*n,n); 
    Gamma_2 = zeros(N1*n,N1*m);
    
    % for joint 2 prediction update
    for r = 1:N1
    
    Phi_2(((r-1)*n)+1:r*n,1:n) = A_2^r;
    
        for c = 1:N1
            if r >= c
            
                Gamma_2(((r-1)*n)+1:r*n,((c-1)*m)+1:c*m) = A_2^(r-c)*B_2;
            
            end
        end
    end
    % predicted model
     X_k_2 = Phi_2*x_k_2 + Gamma_2*u2;
     
    % the basic idea, here, is input optimization that satisfy the minimum cost 
    % the detail algorithm is presented in Dr. Prof. Daniel Görges's model
    % prediction control lectures, Technical university of kaiserslautern
    
    Omega = zeros(N1*n,N1*n);
    Psi = zeros(N1*m,N1*m);
    
    Q = [10 0 0 0;
        0 0.003 0 0;
        0 0 1 0;
        0 0 0 0.001];
    R = 3;
    
    for r = 1:N1

            Omega((r-1)*n+1:r*n,(r-1)*n+1:r*n) = Q;

            Psi((r-1)*m+1:r*m,(r-1)*m+1:r*m) = R;
    
    end
    
    % optimization * objective function*
    
        cost2 = (X_k_2-X_ref_2)'*Omega*(X_k_2-X_ref_2) + u2'*Psi*u2;
   
end