function cost1 = costfunction1(X_ref_1,x_k_1,u1)

% Definition of variables
Ts=0.01;   
g_bar = [1000,2000,2,2,2,260,260,130];
N1 = 10;

% state space model after time delay estimation
    A_1 = [1 Ts 0 0;
        0 2 0 -1;
        1 0 0 0;
        0 1 0 0];
    B_1 = [0; (g_bar(1)*Ts); 0; 0];
    [n, m] = size(B_1);
    
    % the basic idea, here, is prediction of the future state base on the
    % current information and system model 
    % the detail algorithm is presented in Dr. Prof. Daniel Görges's model
    % prediction control lectures, Technical university of kaiserslautern

    Phi_1 = zeros(N1*n,n); 
    Gamma_1 = zeros(N1*n,N1*m);

    % for joint 1 prediction update
    for r = 1:N1
    
    Phi_1(((r-1)*n)+1:r*n,1:n) = A_1^r;
    
        for c = 1:N1
            if r >= c
            
                Gamma_1(((r-1)*n)+1:r*n,((c-1)*m)+1:c*m) = A_1^(r-c)*B_1;
            
            end
        end
    end
 
    % predicted model
    X_k_1 = Phi_1*x_k_1 + Gamma_1*u1;

    % the basic idea, here, is input optimization that satisfy the minimum cost 
    % the detail algorithm is presented in Dr. Prof. Daniel Görges's model
    % prediction control lectures, Technical university of kaiserslautern
    
    Omega = zeros(N1*n,N1*n);
    Psi = zeros(N1*m,N1*m);
    
    Q = [40 0 0 0;
        0 0.12 0 0;
        0 0 5 0;
        0 0 0 0.001];
    R = 3;

    for r = 1:N1

            Omega((r-1)*n+1:r*n,(r-1)*n+1:r*n) = Q;

            Psi((r-1)*m+1:r*m,(r-1)*m+1:r*m) = R;
    
    end

    % optimization * objective function*
    
        cost1 = (X_k_1-X_ref_1)'*Omega*(X_k_1-X_ref_1) + u1'*Psi*u1;

end