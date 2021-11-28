function h = LSFit(xarr,yarr,time)
%% Change polar coordinates to Regular
A=load('handpositionsy.txt');
B=load('handpositionsx.txt');
C=load('handpositionsz.txt');

%% Position - x vs time
XX = xarr';
t =  time'; %0:0.5:200; %%200 is what we put into help code
% XXu=unique(XX, 'stable');
% tu= t';
%create a function that eliminates duplicate points
% 
% figure(2)
plot(t,XX, 'o');
title('X Ball Position over Time')
xlabel('time (sec)')
ylabel('x (m)')

%code below is for when we figure out why our data pts aren't unique
% xxball=0:0.1:200;
% yball=spline(tu,XXu, xxball);
% plot(tu,XXu, 'o', xxball,yball)
% title('X ball position over Time')
% xlabel('time(sec)')
% ylabel('ball position')
% ball_velocity=diff(yball);
% figure(4)
% plot(tu, ball_velocity, 'o')
% title('ball velocity')
% xlabel('time(sec)')
% ylabel('V(m/sec)')
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
figure(3)
xx=0:1:89;
yy=spline(ty,handpositiony, xx);
plot(ty,handpositiony, 'o', xx,yy)
title('Y position right hand over Time')
xlabel('time(sec)')
ylabel('y hand position')
y_velocityhand=diff(yy);
figure(4)
plot(ty, y_velocityhand, 'o')
title('Y-hand velocity')
xlabel('time(sec)')
ylabel('V(m/sec)')
%y hand velocity is derivative of spline
% 
tx=1:length(B);
handpositionx=B;
figure(5)
xxx=0:1:89;
yyy=spline(tx,handpositionx, xxx);
plot(tx,handpositionx, 'o', xxx,yyy)
title('X position right hand over Time')
xlabel('time(sec)')
ylabel('X hand position')
x_velocityhand=diff(yyy);
figure(6)
plot(tx, x_velocityhand, 'o')
title('X-hand velocity')
xlabel('time(sec)') 
ylabel('V(m/sec)')


tz=1:length(C);
handpositionz=C;
figure(7)
xx=0:1:89;
yyyy=spline(tz,handpositionz, xx);
plot(tz,handpositionz, 'o', xx,yyyy)
title('Z position right hand over Time')
xlabel('time(sec)')
ylabel('Z hand position')
z_velocityhand=diff(yyyy);
figure(12)
plot(tz, z_velocityhand, 'o')
title('Z-hand velocity')
xlabel('time(sec)')
ylabel('V(m/sec)')

figure(8)
plot3(handpositionx, handpositiony,tz,'o')
title('x-y parametric position over time hand')
xlabel('x hand position spline')
ylabel('y hand position spline')
zlabel('time')

figure(9)
plot3(handpositionx, handpositionz, tz, '-o')
title('x-z position over time hand')
xlabel('x hand position')
ylabel('z hand position')
zlabel('time')

figure(10)
plot3(handpositiony, handpositiony, tz, '-o')
title('y-z position over time hand')
xlabel('y hand position')
ylabel('z hand position')
zlabel('time')

instantvelocity=0;
for z=1:89
 instantvelocity=instantvelocity+30*sqrt((handpositionx[z]-handpositionx[z-1])^2+(handpositiony[z]-handpositiony[z-1])^2+(handpositionz[z]-handpositionz[z-1])^2));
end
fprintf('%.4f',instantvelocity);
% hand velocity initial
% n=90;vfxs
% instantvelocity=(1/ty)*sqrt((x[n]-x[n-1])^2+(y[n]-y[n-1])^2+
% (z[n]-z[n-1])^2);

