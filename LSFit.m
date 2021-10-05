function h = LSFit(xarr,yarr,time)
%% Change polar coordinates to Regular
%A=load('Positional_Values.txt');

figure(1)
x = xarr'; %A(1,:)';
y = yarr'; %A(2,:)';
plot(x,y,'o');
title('Rectangular Coordinates of Pitch')
xlabel('Distance (m)')
ylabel('Height (m)')
%% Position - x vs time
XX = xarr';
t =  time';

figure(2)
plot(t,XX,'o')
title('X Position over Time')
xlabel('time (sec)')
ylabel('x (m)')

%[B,TF] = rmoutliers(XX,'movmedian',0.05,'SamplePoints',t);
%plot(t,XX,'b.-',t(~TF),B,'r.-')
%legend('Input Data','Output Data')

% figure(3)
% curvefit_x = fit(t,XX,'poly2','normalize','off');
% plot(curvefit_x,t,XX);
% title('Best Fit Curve of X Position over Time')
% xlabel('time (sec)')
% ylabel('x (m)')
% 
% time_final = 7;
% curvefit_x(time_final);
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
% figure(4)
% fx = differentiate(curvefit_x,t);
% plot(t,fx,'o');
% 
% % Get equation of best fit line of velocity
% curvefit_x_velo = fit(t,fx,'poly1','normalize','off');
% plot(curvefit_x_velo,t,fx);
% title('Predicted X-Velocity over Time')
% xlabel('time (sec)')
% ylabel('Vx (m/s)')
% 
% % time_final = 7;
% % curvefit_x_velo(time_final);
% 
% formula(curvefit_x_velo);
% p5 = curvefit_x_velo.p1;
% p6 = curvefit_x_velo.p2;
% %p7 = curvefit_x_velo.p3;
% equation_velo_x = p5*t + p6;
% fprintf('X Velocity Equation: %ft + %f \n', p5,p6);
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

