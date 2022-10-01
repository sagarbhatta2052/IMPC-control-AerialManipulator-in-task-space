
% this code is modified base on my application, the desired and acutal
% trajectory data is updated from the IMPC aerial manipulator simulation
% data and the drone animation is performed from it.


% once the impc simulation data is on workspace, simply copy paste this
% code in the command window and  the desired result will pup up.


% JITENDRA SINGH
% India 
% this code written for making the animation of Quadcoptor model, all units
% are in meters, in this code of example we are using 'HGtransform'
% function for animate the trajectory of quadcopter
% {Thanks to MATLAB}


for k = 0:1:499
 
    x(1,k+1)     = q_t(3,(k+1)*10);
    y(1,k+1)     = q_t(4,(k+1)*10);
    z(1,k+1)     = q_t(5,(k+1)*10);
    
    yaw(1,k+1)   = 0*q_t(3,(k+1)*10);
    roll(1,k+1)  = 0*q_t(3,(k+1)*10);
    pitch(1,k+1) = 0*q_t(3,(k+1)*10);
    
    % actual position of end-effector
    x_1(1,k+1)     = End_Effect_xx(1,(k+1)*10);
    y_1(1,k+1)     = End_Effect_yy(1,(k+1)*10);
    z_1(1,k+1)     = End_Effect_zz(1,(k+1)*10);
    
    % desired position of quadrotor
    x_2(1,k+1)     = q_ref_3(1,(k+1)*10);
    y_2(1,k+1)     = q_ref_4(1,(k+1)*10);
    z_2(1,k+1)     = q_ref_5(1,(k+1)*10);


    % desired position of End-effector
    x_3(1,k+1)     = E_ref_3(1,(k+1)*10);
    y_3(1,k+1)     = E_ref_4(1,(k+1)*10);
    z_3(1,k+1)     = E_ref_5(1,(k+1)*10);
    
    k = k + 1;
end

%% Define design parameters
D2R = pi/180;
R2D = 180/pi;
b   = 0.6;   % the length of total square cover by whole body of quadcopter in meter
a   = b/3;   % the legth of small square base of quadcopter(b/4)
H   = 0.06;  % hight of drone in Z direction (4cm)
H_m = H+H/2; % hight of motor in z direction (5 cm)
r_p = b/4;   % radius of propeller
%% Conversions
ro = 45*D2R;                   % angle by which rotate the base of quadcopter
Ri = [cos(ro) -sin(ro) 0;
      sin(ro) cos(ro)  0;
       0       0       1];     % rotation matrix to rotate the coordinates of base 
base_co = [-a/2  a/2 a/2 -a/2; % Coordinates of Base 
           -a/2 -a/2 a/2 a/2;
             0    0   0   0];
base = Ri*base_co;             % rotate base Coordinates by 45 degree 
to = linspace(0, 2*pi);
xp = r_p*cos(to);
yp = r_p*sin(to);
zp = zeros(1,length(to));
%% Define Figure plot
 fig1 = figure('pos', [0 50 800 600]);
 hg   = gca;
 view(68,53);
 grid on;
 axis equal;
 xlim([-3 3]); ylim([-3 3]); zlim([0 4]);
 title('Drone Animation')
 xlabel('X[m]');
 ylabel('Y[m]');
 zlabel('Z[m]');
 hold(gca, 'on');
 
%% Design Different parts
% design the base square
 drone(1) = patch([base(1,:)],[base(2,:)],[base(3,:)],'r');
 drone(2) = patch([base(1,:)],[base(2,:)],[base(3,:)+H],'r');
 alpha(drone(1:2),0.7);
% design 2 parpendiculer legs of quadcopter 
 [xcylinder ycylinder zcylinder] = cylinder([H/2 H/2]);
 drone(3) =  surface(b*zcylinder-b/2,ycylinder,xcylinder+H/2,'facecolor','b');
 drone(4) =  surface(ycylinder,b*zcylinder-b/2,xcylinder+H/2,'facecolor','b') ; 
 alpha(drone(3:4),0.6);
% design 4 cylindrical motors 
 drone(5) = surface(xcylinder+b/2,ycylinder,H_m*zcylinder+H/2,'facecolor','r');
 drone(6) = surface(xcylinder-b/2,ycylinder,H_m*zcylinder+H/2,'facecolor','r');
 drone(7) = surface(xcylinder,ycylinder+b/2,H_m*zcylinder+H/2,'facecolor','r');
 drone(8) = surface(xcylinder,ycylinder-b/2,H_m*zcylinder+H/2,'facecolor','r');
 alpha(drone(5:8),0.7);
% design 4 propellers
 drone(9)  = patch(xp+b/2,yp,zp+(H_m+H/2),'c','LineWidth',0.5);
 drone(10) = patch(xp-b/2,yp,zp+(H_m+H/2),'c','LineWidth',0.5);
 drone(11) = patch(xp,yp+b/2,zp+(H_m+H/2),'p','LineWidth',0.5);
 drone(12) = patch(xp,yp-b/2,zp+(H_m+H/2),'p','LineWidth',0.5);
 alpha(drone(9:12),0.3);
 
 %% design the robot manipulator 

 % design of first link
 drone(13) = surface(xcylinder,ycylinder,-1.8*H_m*zcylinder+H/2,'facecolor','r');
 
 % design the second link
 drone(14) =  surface(0.34*b*zcylinder,ycylinder,xcylinder+H/2-0.14,'facecolor','b');
 
 % design the third link
 drone(15) = surface(xcylinder+0.18,ycylinder,-1.8*H_m*zcylinder+H/2-0.14,'facecolor','r');
 

%% create a group object and parent surface
  combinedobject = hgtransform('parent',hg );
  set(drone,'parent',combinedobject)
%  drawnow

plot3(x_2,y_2,z_2, 'c--','LineWidth',1.5);
plot3(x_3,y_3,z_3, 'm--','LineWidth',1.5);

 
 for i = 1:length(x)
  
     ba = plot3(x(1:i),y(1:i),z(1:i), 'r','LineWidth',1.5);
   
     ba_1 = plot3(x_1(1:i),y_1(1:i),z_1(1:i), 'b','LineWidth',1.5);
     
     translation = makehgtform('translate',...
                               [x(i) y(i) z(i)]);
     %set(combinedobject, 'matrix',translation);
     rotation1 = makehgtform('xrotate',(pi/180)*(roll(i)));
     rotation2 = makehgtform('yrotate',(pi/180)*(pitch(i)));
     rotation3 = makehgtform('zrotate',yaw(i));
     %scaling = makehgtform('scale',1-i/20);
     set(combinedobject,'matrix',...
          translation*rotation3*rotation2*rotation1);
      

      F(i) =  getframe(gcf);
        %delete(b);
     drawnow
 
 end
 
%Open the VideoWriter object, write the movie, and class the file
video =  VideoWriter('drone_Animation','MPEG-4');
video.FrameRate = 60;
open(video)
writeVideo(video, F);
close(video)
