function cost4 = costfunction4(X_ref_4,x_k_4,u4)
    
% Definition of variables
Ts=0.01;      
g_bar = [1000,2000,2,2,2,260,260,130];
N1 = 10;

% state space model after time delay estimation
    A_4 = [1 Ts 0 0;
        0 2 0 -1;
        1 0 0 0;
        0 1 0 0];
    B_4 = [0; (g_bar(4)*Ts); 0; 0];
    [n, m] = size(B_4);
    
    % the basic idea, here, is prediction of the future state base on the
    % current information and system model 
    % the detail algorithm is presented in Dr. Prof. Daniel Görges's model
    % prediction control lectures, Technical university of kaiserslautern
    
    Phi_3 = zeros(N1*n,n); 
    Gamma_3 = zeros(N1*n,N1*m);
   
  % for joint 3 prediction update
        for r = 1:N1
    
        Phi_3(((r-1)*n)+1:r*n,1:n) = A_4^r;
    
            for c = 1:N1
                if r >= c
            
                Gamma_3(((r-1)*n)+1:r*n,((c-1)*m)+1:c*m) = A_4^(r-c)*B_4;
            
                end
            end
        end
    % predicted model
         X_k_4 = Phi_3*x_k_4 + Gamma_3*u4;
         
    % the basic idea, here, is input optimization that satisfy the minimum cost 
    % the detail algorithm is presented in Dr. Prof. Daniel Görges's model
    % prediction control lectures, Technical university of kaiserslautern
    
    Omega = zeros(N1*n,N1*n);
    Psi = zeros(N1*m,N1*m);
    
    Q = [100 0 0 0;
        0 1 0 0;
        0 0 10 0;
        0 0 0 0.1];
    R = 1;
    
    for r = 1:N1

            Omega((r-1)*n+1:r*n,(r-1)*n+1:r*n) = Q;

            Psi((r-1)*m+1:r*m,(r-1)*m+1:r*m) = R;
    
    end
        
    % optimization * objective function*
    
        cost4 = (X_k_4-X_ref_4)'*Omega*(X_k_4-X_ref_4) + u4'*Psi*u4;
   
end
