function h = LSFit(xarr,yarr,time)
%% Change polar coordinates to Regular
A=load('handpositionsy.txt');
B=load('handpositionsx.txt');
C=load('handpositionsz.txt');

%% Position - x vs time
XX = xarr';
t =  time';
figure(2)
plot(t,XX, 'o');
title('X Position over Time')
xlabel('time (sec)')
ylabel('x (m)')

% curvefit_x(t);
% 
% formula(curvefit_x);
% p1 = curvefit_x.p1;
% p2 = curvefit_x.p2;
% p3 = curvefit_x.p3;
% %p4 = curvefit_x.p4;
% equation_x = p1*(t.^2) + p2*t + p3; % + p4; 
% fprintf('X Equation: %ft^2 + %ft + %f \n', p1,p2,p3);%,p4);
% 
% % Differentiate to get the predicted velocity in x direction
% figure(2)
% fx = differentiate(curvefit_x,t);
% plot(t,fx,'o');
% %fxx=differentiate(XX,t);
% 
% % Get equation of best fit line of velocity
% curvefit_x_velo = fit(t,fx,'poly1','normalize','off');
% figure(12)
% plot(curvefit_x_velo,t,fx);
% title('Predicted X-Velocity over Time')
% xlabel('time (sec)')
% ylabel('Vx (m/s)')

% % after this, add eqns for rotational velocity
%x=r(wt+sinwt) for position where r is the radius, where w is the
%angular velocity
%rot_velocity=rw(1+coswt)
%solve for w
%
% radius_ball=0.1016; %4inches)

%% Hand position calculations
%figure out how to read time data into matlab from camera
ty=1:length(A);
handpositiony=A;
figure(5)
plot(ty,handpositiony, 'o')
title('Y position right hand over Time')
xlabel('time(sec)')
ylabel('y hand position')

tx=1:length(B);
handpositionx=B;
figure(6)
plot(tx,handpositionx, 'o')
title('x position right hand over Time')
xlabel('time(sec)')
ylabel('x hand position')

tz=1:length(C);
handpositionz=C;
figure(7)
plot(tz,handpositionz, 'o')
title('z position right hand over Time')
xlabel('time(sec)')
ylabel('z hand position')


figure(8)
plot3(handpositionx, handpositiony, handpositionz, '-o')
title('x-y-z positioning hand')
xlabel('x hand position')
ylabel('y hand position')
zlabel('z hand position')

figure(9)
plot3(handpositionx, handpositiony, tz, '-o')
title('x-y position over time hand')
xlabel('x hand position')
ylabel('y hand position')
zlabel('time')

figure(10)
plot3(handpositionx, handpositionz, tz, '-o')
title('x-z position over time hand')
xlabel('x hand position')
ylabel('z hand position')
zlabel('time')

figure(11)
plot3(handpositiony, handpositiony, tz, '-o')
title('y-z position over time hand')
xlabel('y hand position')
ylabel('z hand position')
zlabel('time')

%hand velocity initial
%n=90;
%instantvelocity=(1/ty)*sqrt((x[n]-x[n-1])^2+(y[n]-y[n-1])^2+
%(z[n]-z[n-1])^2);

%[B,TF] = rmoutliers(XX,'movmedian',0.05,'SamplePoints',t);
%plot(t,XX,'b.-',t(~TF),B,'r.-')
%legend('Input Data','Output Data')


% 
% %% Position - y vs time
% YY = yarr';
% t = time';
% figure(5)
% plot(t,YY,'o')
% title('Y Position over Time')
% xlabel('time (sec)')
% ylabel('y (m)')
% 
% figure(6)
% curvefit_y = fit(t,YY,'poly2','normalize','off');
% plot(curvefit_y,t,YY);
% title('Best Fit Curve of Y Position over Time')
% xlabel('time (sec)')
% ylabel('y (m)')
% 
% time_final = 7;
% curvefit_y(time_final);
% 
% formula(curvefit_y);
% p8 = curvefit_y.p1;
% p9 = curvefit_y.p2;
% p10 = curvefit_y.p3;
% %p11 = curvefit_y.p4;
% equation_y = p8*(t.^2) + p9*t + p10;
% fprintf('Y Equation: %ft^2 + %ft + %f \n', p8,p9,p10);
% 
% % Differentiate to get the predicted velocity in y direction
% figure(7)
% fy = differentiate(curvefit_y,t);
% plot(t,fy,'o');
% 
% % Get equation of least squares fit line
% curvefit_y_velo = fit(t,fy,'poly1','normalize','off');
% plot(curvefit_y_velo,t,fy);
% title('Predicted Y-Velocity over Time')
% xlabel('time (sec)')
% ylabel('Vy (m/s)')
% 
% time_final = 7;
% curvefit_y_velo(time_final);
% 
% formula(curvefit_y_velo);
% p12 = curvefit_y_velo.p1;
% p13 = curvefit_y_velo.p2;
% %p14 = curvefit_y_velo.p3;
% equation_velo_y = p12*t + p13;
% fprintf('Y Velocity Equation: %ft + %f \n', p12,p13);
% 
% %% Predicted Velocity without Magnitude
% velocity = sqrt(((fx).^2) + ((fy).^2));
% figure(8)
% plot(t,velocity, 'o')
% title('Velocity over Time')
% xlabel('time (sec)')
% ylabel('Velocity')
%  
% % Get equation of least squares fit line
% curvefit_velo = fit(t,velocity,'poly1','normalize','off');
% plot(curvefit_velo,t,velocity);
% title('Predicted Velocity over Time')
% xlabel('time (sec)')
% ylabel('Velocity (m/s)')
%  
% time_final = 7;
% curvefit_velo(time_final);
%  
% formula(curvefit_velo);
% p15 = curvefit_velo.p1;
% p16 = curvefit_velo.p2;
% %p17 = curvefit_velo.p3;
% equation_velo = p15*t + p16;
% fprintf('Velocity Equation: %ft + %f \n', p15,p16);
% 
% %% Predicted Acceleration
% % Differentiate to get the predicted acceleration
% figure(9)
% facc = differentiate(curvefit_velo,t);
% plot(t,facc,'o');
% 
% % Get equation of least squares fit line
% curvefit_acc = fit(t,facc,'poly1','normalize','off');
% plot(curvefit_acc,t,facc);
% title('Predicted Acceleration over Time')
% xlabel('time (sec)')
% ylabel('Acceleration (m/s^2)')
% 
% time_final = 7;
% curvefit_acc(time_final);
% 
% formula(curvefit_acc);
% p18 = curvefit_acc.p1;
% p19 = curvefit_acc.p2;
% %p20 = curvefit_acc.p3;
% equation_acc = p19;
% fprintf('Acceleration Equation: %f \n', p19);


% %% Final Time
% 
% xf = 252; %inches
% xfm = (xf/39.37) - x(1); %meters
% c = -xfm;
% v0 = p6;
% a0 = p5;
% a = a0/2;
% tf1 = (-v0+sqrt((v0^2)-(4*a*c)))/(2*a);
% tf2 = (-v0-sqrt((v0^2)-(4*a*c)))/(2*a);
% 
% tf = min(abs(tf1),abs(tf2));
% 
% h = p8*(tf^2) + p9*tf + p10;
% fprintf('%g %g %g \n',tf1,tf2,h)

