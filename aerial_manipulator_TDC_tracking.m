%%% the idea of this code is the implementation of incremental model
%%% predictive control for aerial manipulator in the task space*, The
%%% purposed reference end effector trajectories is first map from the end
%%% variables to the joint variables. After getting the joint variables, the joint space control
%%% strategies is performed that basically minimized the end effector
%%% variables.

%% clear and close all the warning
clc;
clear;
warning off;

%% Definition of variables

End_time = 35;                  % total time intervel
Ts=0.01;                        % sampling time

%% initialization of states and input variables
% q = [theta1, theta2, X, Y, Z, Phi, Theta, Psi]'

q = [pi/2,0,0,0,2.35,0,0,0]';                    % initial position
qd = [0,0,0,0,0,0,0,0]';                        % initial velocity
qdd = [0,0,0,0,0,0,0,0]';                       % initial acceleration

% Torque inputs :- two for robot manipulator and four for quadrotor
% T = [Ts1, Ts2, u1, u2, u3, u4]';

T = [0,0,0,0,0,0]';    
ux = 0';
uy = 0';
uz = 0';

% Define the actual system states for the plots for the whole simulation range
q_t = zeros(8,End_time/Ts+1);
qd_t = zeros(8,End_time/Ts+1);
qdd_t = zeros(8,End_time/Ts+1);
q_t(:,1) = q;
qd_t(:,1) = qd;
qdd_t(:,1) = qdd;

u = zeros(8,End_time/Ts+1);
T_t = zeros(6,End_time/Ts+1);

%% Define the reference system states for the plots for the whole simulation range
q_ref__1 = 0*ones(1,End_time/Ts+1);
q_ref__2 = 0*ones(1,End_time/Ts+1);
q_ref__3 = 0*ones(1,End_time/Ts+1);
q_ref__4 = 0*ones(1,End_time/Ts+1);
q_ref__5 = 0*ones(1,End_time/Ts+1);
q_ref__6 = 0*ones(1,End_time/Ts+1);
q_ref__7 = 0*ones(1,End_time/Ts+1);
q_ref__8 = 0*ones(1,End_time/Ts+1);

qd_ref_1 = 0*ones(1,End_time/Ts+1);
qd_ref_2 = 0*ones(1,End_time/Ts+1);
qd_ref_3 = 0*ones(1,End_time/Ts+1);
qd_ref_4 = 0*ones(1,End_time/Ts+1);
qd_ref_5 = 0*ones(1,End_time/Ts+1);
qd_ref_6 = 0*ones(1,End_time/Ts+1);
qd_ref_7 = 0*ones(1,End_time/Ts+1);
qd_ref_8 = 0*ones(1,End_time/Ts+1);

qdd_ref_1 = 0*ones(1,End_time/Ts+1);
qdd_ref_2 = 0*ones(1,End_time/Ts+1);
qdd_ref_3 = 0*ones(1,End_time/Ts+1);
qdd_ref_4 = 0*ones(1,End_time/Ts+1);
qdd_ref_5 = 0*ones(1,End_time/Ts+1);
qdd_ref_6 = 0*ones(1,End_time/Ts+1);
qdd_ref_7 = 0*ones(1,End_time/Ts+1);
qdd_ref_8 = 0*ones(1,End_time/Ts+1);

Phi_r = 0*ones(1,End_time/Ts+1);
Theta_r = 0*ones(1,End_time/Ts+1);

% update the initial reference states for the aerial manipulator

q_ref_1(1,1) = pi/2;
q_ref_2(1,1) = 0;
q_ref_3(1,1) = 0;
q_ref_4(1,1) = 0;
q_ref_5(1,1) = 2.35;


%% In this part we defined the reference trajectory for the end effector, 
%%% then performed the inverse kinematics to find the joints angles and 
%%% the reference position of the quadrotor.

k = 1;
   for t = 0:Ts:End_time+10*Ts

        E_ref_3(1,1) = 0; 
        E_ref_3(1,k+1) = 2*sin(0.2*t);

        
        E_ref_4(1,1) = 0;
        E_ref_4(1,k+1) = 2*2*cos(0.2*t)*sin(0.2*t);

        
        E_ref_5(1,1) = 2;
        E_ref_5(1,k+1) = 2 + 0*t;


        % link lengths
        L0 = 0.05;
        L1 = 0.1;
        L2 = 0.2; 

        %% base on the end position and the robotic arm configuration, the inverse kinematics for joints angle

            q_ref_1(1,k+1) = atan2(E_ref_5(1,k+1),E_ref_4(1,k+1));
            q_ref_2(1,k+1) = atan2(E_ref_3(1,k+1),(sqrt(E_ref_5(1,k+1)^2 + E_ref_4(1,k+1)^2)-L1));

        %% inverse kinematics to find the quadrotor position

            q_ref_3(1,k+1) = E_ref_3(1,k+1) - L2*cos(q_ref_1(1,k+1))*sin(q_ref_2(1,k+1));
            q_ref_4(1,k+1) = E_ref_4(1,k+1) - (L1 + L2*cos(q_ref_2(1,k+1)))*cos(q_ref_1(1,k+1));
            q_ref_5(1,k+1) = E_ref_5(1,k+1) + L0 +(L1 + L2*cos(q_ref_2(1,k+1)))*sin(q_ref_1(1,k+1));
            
            k = k + 1;
    
   end

%         plot3(E_ref_3,E_ref_4,E_ref_5)
%         hold on
%         plot3(q_ref_3,q_ref_4,q_ref_5)
        
        
%%% till yet we defined the reference joint variables, theta1 and theta2, and
%%% quadrotor position, x,y,z, base on the end effector position. Now, we
%%% use this references values and perform the joint space control.


k = 2;    
% k value should start form 2 because, in the complete TDE state space model, we use current and last sampling time state. 
% for example: x(k+1) = Ax(k)+ Bu(k) where, x_k_1 =[q_t(1,k); qd_t(1,k); q_t(1,k-1); qd_t(1,k-1)]

for t=0:Ts:End_time-Ts

    %% System dynamic
    qdd = aerial_manipulator_model(T,q,qd);
    qd = qd + qdd*Ts;
    q = q + qd*Ts;

    q_t(:,k) = q;
    qd_t(:,k) = qd;
    qdd_t(:,k) = qdd;
   
    %% Reference Update for phi and theta
    q_ref_6(k) = Phi_r(k);
    q_ref_7(k) = Theta_r(k);
    
%%  IMPC controller design for the aerial manipulator
% Definition of variables for MPC

        g_bar = [1000,2000,2,2,2,260,260,130];              % inverse inertia matrix
        N1 = 10;                                            % length prediction horizon 

%% start of first loop optimization
%% Inputs and  References for IMPC control

        u_1 = zeros(1,N1)';
        u_2 = zeros(1,N1)';
        u_3 = zeros(1,N1)';
        u_4 = zeros(1,N1)';
        u_5 = zeros(1,N1)';
        u_6 = zeros(1,N1)';
        u_7 = zeros(1,N1)';
        u_8 = zeros(1,N1)';

        X_ref_1 = zeros(40,1); 
        X_ref_2 = zeros(40,1);
        X_ref_3 = zeros(40,1);
        X_ref_4 = zeros(40,1);
        X_ref_5 = zeros(40,1);
        X_ref_6 = zeros(40,1);
        X_ref_7 = zeros(40,1);
        X_ref_8 = zeros(40,1);
        
        % every simulation, update only 10 values to perform IMPC for length 10

        for i = 0:N1-1
            
                q_ref_1_1 = q_ref_1(1,(k+i+1));
                q_ref_1_2 = q_ref_1(1,(k+i));
                qd_ref_1_1 = 0;
                qd_ref_1_2 = 0;

                q_ref_2_1 = q_ref_2(1,(k+i+1));
                q_ref_2_2 = q_ref_2(1,(k+i));
                qd_ref_2_1 = 0;
                qd_ref_2_2 = 0;
        
                q_ref_3_1 = q_ref_3(1,(k+i+1));
                q_ref_3_2 = q_ref_3(1,(k+i));
                qd_ref_3_1 = 0;
                qd_ref_3_2 = 0;

                q_ref_4_1 = q_ref_4(1,(k+i+1));
                q_ref_4_2 = q_ref_4(1,(k+i));
                qd_ref_4_1 = 0;
                qd_ref_4_2 = 0;

                q_ref_5_1 = q_ref_5(1,(k+i+1));  
                q_ref_5_2 = q_ref_5(1,(k+i));
                qd_ref_5_1 = 0;
                qd_ref_5_2 = 0;

                X_ref_1((1+(i*4)):(4+(i*4)),1) = [q_ref_1_1, qd_ref_1_1, q_ref_1_2, qd_ref_1_2]';
                X_ref_2((1+(i*4)):(4+(i*4)),1) = [q_ref_2_1, qd_ref_2_1, q_ref_2_2, qd_ref_2_2]';
                X_ref_3((1+(i*4)):(4+(i*4)),1) = [q_ref_3_1, qd_ref_3_1, q_ref_3_2, qd_ref_3_2]';
                X_ref_4((1+(i*4)):(4+(i*4)),1) = [q_ref_4_1, qd_ref_4_1, q_ref_4_2, qd_ref_4_2]';
                X_ref_5((1+(i*4)):(4+(i*4)),1) = [q_ref_5_1, qd_ref_5_1, q_ref_5_2, qd_ref_5_2]';

        end

                %updated references for 10 step horizon for joint angles and position
                    
                X_ref_1;
                X_ref_2;
                X_ref_3;
                X_ref_4;
                X_ref_5;
    
        %current states x(k) for mpc controller
        
        x_k_1 =[q_t(1,k); qd_t(1,k); q_t(1,k-1); qd_t(1,k-1)];
        x_k_2 =[q_t(2,k); qd_t(2,k); q_t(2,k-1); qd_t(2,k-1)];
        x_k_3 =[q_t(3,k); qd_t(3,k); q_t(3,k-1); qd_t(3,k-1)];
        x_k_4 =[q_t(4,k); qd_t(4,k); q_t(4,k-1); qd_t(4,k-1)];
        x_k_5 =[q_t(5,k); qd_t(5,k); q_t(5,k-1); qd_t(5,k-1)];
        x_k_8 =[q_t(8,k); qd_t(8,k); q_t(8,k-1); qd_t(8,k-1)];
    
            %% optimization:- the optimization is perform based on optimization tool fmincon

                    deltau_1 = fmincon(@(u1)costfunction1(X_ref_1,x_k_1,u1),u_1,[],[]);
                    deltau_2 = fmincon(@(u2)costfunction2(X_ref_2,x_k_2,u2),u_2,[],[]);
                    deltau_3 = fmincon(@(u3)costfunction3(X_ref_3,x_k_3,u3),u_3,[],[]);
                    deltau_4 = fmincon(@(u4)costfunction4(X_ref_4,x_k_4,u4),u_4,[],[]);
                    deltau_5 = fmincon(@(u5)costfunction5(X_ref_5,x_k_5,u5),u_5,[],[]);
                    deltau_8 = fmincon(@(u8)costfunction8(X_ref_8,x_k_8,u8),u_8,[],[]);

        % update inputs for the next optimization 
        
        u_1 = [deltau_1(2:end);0];
        u_2 = [deltau_2(2:end);0];
        u_3 = [deltau_3(2:end);0];
        u_4 = [deltau_4(2:end);0];
        u_5 = [deltau_5(2:end);0];
        u_8 = [deltau_8(2:end);0];

        % optimized input for aerial manipulator
        
        u(1,k) = u(1,k-1) + deltau_1(1,1);
        u(2,k) = u(2,k-1) + deltau_2(1,1);
        u(3,k) = u(3,k-1) + deltau_3(1,1);
        u(4,k) = u(4,k-1) + deltau_4(1,1);
        u(5,k) = u(5,k-1) + deltau_5(1,1);
        u(8,k) = u(8,k-1) + deltau_8(1,1);

        %input for position control (ux,uy,uz => u1)
        
        ux = u(3,k);
        uy = u(4,k);
        uz = u(5,k);
        u1 = sqrt(ux^2+uy^2+(uz+0.486*9.81)^2);
        
%% end of first loop optimization

%% start of second loop optimization
%%% here, we try to find out the 10 horizon reference for phi and theta.
%%% Each optimized inputs able to find the corresponding phi and theta reference values.
%%% the detail theories and block diagram of the auxiliary inputs is in the references papers.

        % All delta inputs for 10 horizon to find phi and theta reference 
        
        deltau_3_3 = tril(ones(10)) * deltau_3;
        deltau_4_4 = tril(ones(10)) * deltau_4;
        deltau_5_5 = tril(ones(10)) * deltau_5;
        psi = [0,0,0,0,0,0,0,0,0,0];

        %%% each deltau_x_x produce the corresponding step inputs:
        %%% (ux,uy,uz) and the reference psi is considered as zero

            for r = 0:N1-1
        
                u(3,k+r) = u(3,k-1) + deltau_3_3(r+1,1);
                u(4,k+r) = u(4,k-1) + deltau_4_4(r+1,1);
                u(5,k+r) = u(5,k-1) + deltau_5_5(r+1,1);

                ux = u(3,k+r);
                uy = u(4,k+r);
                uz = u(5,k+r);
                Psi = psi(1,r+1);   

                Phi_r(k+r+1) = asin((ux*sin(Psi)-uy*cos(Psi))/sqrt(ux^2+uy^2+(uz+0.486*9.81)^2));
                Theta_r(k+r+1) = atan((ux*cos(Psi)+uy*sin(Psi))/(uz+0.486*9.81));

                q_ref_6(k+r+1) = Phi_r(k+r+1);
                q_ref_7(k+r+1) = Theta_r(k+r+1);

                qd_ref_6(k+r+1) = 0;
                qd_ref_7(k+r+1) = 0;

                X_ref_6((1+(r*4)):(4+(r*4)),1) = [q_ref_6(k+r+1), qd_ref_6(k+r+1), q_ref_6(k+r), qd_ref_6(k+r)]';
                X_ref_7((1+(r*4)):(4+(r*4)),1) = [q_ref_7(k+r+1), qd_ref_7(k+r+1), q_ref_7(k+r), qd_ref_7(k+r)]';
    
            end
    
                % updated psi and theta 
                
                X_ref_6;
                X_ref_7;
    
                % current states x(k) for psi and theta for mpc controller
                
                x_k_6 =[q_t(6,k); qd_t(6,k); q_t(6,k-1); qd_t(6,k-1)];
                x_k_7 =[q_t(7,k); qd_t(7,k); q_t(7,k-1); qd_t(7,k-1)];
     
                %% optimization:- the optimization is perform based on optimization tool fmincon
                
                            deltau_6 = fmincon(@(u6)costfunction6(X_ref_6,x_k_6,u6),u_6,[],[]);
                            deltau_7 = fmincon(@(u7)costfunction7(X_ref_7,x_k_7,u7),u_7,[],[]);
   
                % update inputs for the next optimization 
                
                u_6 = [deltau_6(2:end);0];
                u_7 = [deltau_7(2:end);0];
                
                % optimized input for aerial manipulator

                u(6,k) = u(6,k-1) + deltau_6(1,1);
                u(7,k) = u(7,k-1) + deltau_7(1,1);

%% end of second optimization
      
                %% updated auxiliary inputs
                
                Psi = q(8);
                Phi = q(6);
                Theta = q(7);

                ux = (cos(Phi)*sin(Theta)*cos(Psi)+sin(Phi)*sin(Psi))*u1;
                uy = (cos(Phi)*sin(Theta)*sin(Psi)-sin(Phi)*cos(Psi))*u1;
                uz = -0.486*9.81 + cos(Phi)*cos(Theta)*u1;

                u(3:5,k) = [ux;uy;uz];

                %% Finalized actual inputs for aerial manipulator for the next simulation
                
                T = [u(1:2,k); u1; u(6:8,k)];
                T_t(:,k) = T;
    
    k = k + 1

end

% final end-effector position
L0 = 0.05;
L1 = 0.1;
L2 = 0.2;
End_Effect_xx = q_t(3,:) + L2.*cos(q_t(1,:)).*sin(q_t(2,:));
End_Effect_yy = q_t(4,:) + (L1 + L2.*cos(q_t(2,:))).*cos(q_t(1,:));
End_Effect_zz = q_t(5,:) - L0 -(L1 + L2.*cos(q_t(2,:))).*sin(q_t(1,:));

%% reference tracking plot
figure
plot3(q_t(3,:),q_t(4,:),q_t(5,:));
hold on
plot3(q_ref_3,q_ref_4,q_ref_5,'--');
hold on 
plot3( E_ref_3(1,:),E_ref_4(1,:),E_ref_5(1,:))
hold on
plot3( End_Effect_xx,End_Effect_yy,End_Effect_zz,'--')

%% error plot
time_serial = 0:Ts:End_time;
figure
subplot(3,1,1)
%plot x error over time horizon
plot( 0:Ts:End_time , q_t(3,:) - q_ref_3(1,1:5001) );
title('X error')
xlabel('time')
ylabel('amplitude')

subplot(3,1,2)
%plot y error over time horizon
plot( 0:Ts:End_time , q_t(4,:) - q_ref_4(1,1:5001) );
title('Y error')
xlabel('time')
ylabel('amplitude')

subplot(3,1,3)
%plot z error over time horizon
plot( 0:Ts:End_time , q_t(5,:) - q_ref_5(1,1:5001));
title('Z error')
xlabel('time')
ylabel('amplitude')

%% joint error plot
figure
subplot(2,1,1)
%plot thita1 error over time horizon
plot( 0:Ts:End_time , q_t(1,:) - q_ref_1(1,1:5001) );
title('\theta1 error')
xlabel('time')
ylabel('amplitude(Grad)')

subplot(2,1,2)
%plot thita2 error over time horizon
plot( 0:Ts:End_time , q_t(2,:) - q_ref_2(1,1:5001) );
title('\theta2 error')
xlabel('time')
ylabel('amplitude(Grad)')


%% error plot for  end effector
time_serial = 0:Ts:End_time;
figure
subplot(3,1,1)
%plot x error over time horizon
plot( 0:Ts:End_time, End_Effect_xx(1,:) - E_ref_3(1,1:5001) );
title('Xe axis error')
xlabel('time(s)')
ylabel('amplitude(m)')

subplot(3,1,2)
%plot y error over time horizon
plot( 0:Ts:End_time, End_Effect_yy(1,:) - E_ref_4(1,1:5001) );
title('Ye axis error')
xlabel('time(s)')
ylabel('amplitude(m)')

subplot(3,1,3)
%plot z error over time horizon
plot( 0:Ts:End_time, End_Effect_zz(1,:) - E_ref_5(1,1:5001) );
title('Ze axis error')
xlabel('time(s)')
ylabel('amplitude(m)')

set(gcf,'color','w');

