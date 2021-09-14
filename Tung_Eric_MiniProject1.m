% Eric Tung - Code for Mini-Project 1
clear all; close all;

% Define robot as a planar circular shape
% zr = small height 
% radius = robot radius
radius=.3;
zr=.1;
robot = collisionCylinder(radius,zr);

% position the robot in some initial configuration
p0=[1.55;4];th0=pi;
robot.Pose(1:2,4)=p0;
robot.Pose(1:2,1:2)=rot2(th0);

% h1 = figure handle
h1=showroom(1,5,5); % see showroom function below
% hold the figure so you can plot over it
hold on 

% specify a target pose
p1=[1.55;2.5];
% length of straight line path
lf1= norm(p1-p0);
% generate a lambda vector
N=4;
l1=(0:lf1/N:lf1);
% generate the path based on lambda
path1 = p0*(1-l1/lf1) + p1*l1/lf1;
% # of elements in l1
n1=length(l1);
% show the straight line path 
for i=1:n1;robot.Pose(1:2,4)=path1(:,i);show(robot);end
%*****************************************************************************************************************************

%***********************************
% generate a second straight line path
%***********************************
% generate a path to the second target
p2=[1;1.1];
lf2=norm(p2-p1);
l2=(lf2/N:lf2/N:lf2);
n2=length(l2);
path2 = p1*(1-l2/lf2) + p2*l2/lf2;

%***********************************
% combine the two paths into a single path
%***********************************
% complete l
l=[l1 l2+l1(end)];
nl=length(l);
path12=[path1 path2];
% show the complete path in anothe figure
h2=showroom(2,5,5);hold on
for i=1:length(l);robot.Pose(1:2,4)=path12(:,i);show(robot);end
%*****************************************************************************************************************************

%***********************************
% generate theta path
%***********************************

theta0=[pi];
theta1=[3*pi/2];
deltatheta=norm(p2-p0);
t2=(0:deltatheta/(nl-1):deltatheta);
thetapath_short = theta0*(1-t2/deltatheta) + theta1*t2/deltatheta;
thetavectors_short = getxy(thetapath_short);
h100 = showroom(100,5,5);
q = quiver(path12(1,:),path12(2,:),thetavectors_short(1,:),thetavectors_short(2,:),0.3);
q.LineWidth = 1.2;

%***********************************
% avoid the kink in the path (adjust n_p and n_a and see what happens)
%***********************************
% spline the two together
% # of points on the path in the spline region from the break point
n_p = 4;
% # of points to include in the spline (n_a close to n_p means closer path
% following)
n_a = 1;
% # of segments in spline region
n_s = 10;
% l_s is no longer the path length
l_s = [l(n1-n_p):(l(n1+n_p)-l(n1-n_p))/n_s:l(n1+n_p)];
p_s = spline([l(n1-n_p:n1-n_p+n_a) l(n1+n_p-n_a:n1+n_p)],...
    path12(:,[n1-n_p:n1-n_p+n_a,n1+n_p-n_a:n1+n_p]),l_s);
% show the path interpolation
figure(10);plot(l,path12,l_s,p_s,'linewidth',2);
legend('x','y','x with spline','y with spline');
title('robot position vs. path length');
xlabel('lambda (m)');ylabel('x-y position of robot (m)')

% this is the combined path
l4 = [l(1:n1-n_p-1) l_s l(n1+n_p+1:nl)]; 
pathtotal = [path12(:,1:n1-n_p-1) p_s path12(:,n1+n_p+1:nl)];
% show the complete path in anothe figure
%h4=showroom(4,5,5);hold on
%for i=1:length(l4);robot.Pose(1:2,4)=pathtotal(:,i);show(robot);end

%***********************************
% index path with time (for straight line paths)
%*****************************************************************************************************************************************
%*****************************************************************************************************************************************
%*****************************************************************************************************************************************
% maximum velocity of the vehicle 
umax=[.2;.2];
% max angular velocity of vehicle
wmax = 0.1;
% max speed for segment 1
path1_prime=(p1-p0)/lf1;
l1dotmax=min(abs(umax./path1_prime));
T1 = lf1/l1dotmax;
% max speed of segment 2
path2_prime=(p2-p1)/lf2;
l2dotmax=min(abs(umax./path2_prime));
T2 = lf2/l2dotmax;

%***********************************
% use vehicle kinematics to generate motion
%***********************************
% note that lambda_dot is chosen so that velocity limits are not violated
ts=.1; % 0.1 second sampling period
t1=(0:ts:T1);NT1=length(t1);
pt1=zeros(2,NT1);pt1(:,1)=p0;

% use simple finite difference to propagate robot motion
% assuming you can control the robot velocity
% first segment
for i=1:NT1-1
    pt1(:,i+1)=pt1(:,i)+ts*owmr(pt1(:,i),path1_prime*l1dotmax,umax,-umax);
end
% second segment
t2=(T1:ts:T1+T2);NT2=length(t2);
pt2=zeros(2,NT2);pt2(:,1)=pt1(:,NT1);
for i=1:NT2-1
    pt2(:,i+1)=pt2(:,i)+ts*owmr(pt2(:,i),path2_prime*l2dotmax,umax,-umax);
end
% combine motion
t=[t1 t2(2:NT2)];
pt=[pt1 pt2(:,2:NT2)];
% calculate path length
lt=[0 cumsum(vecnorm(diff(pt')'))];
% motion vs. time 
figure(11);plot(t,pt,'linewidth',2);legend('x','y');
xlabel('time (sec)');xlabel('robot position (m)');
% lambda vs. time (piecewise linear, with a kink at transition)
figure(12);plot(t,lt,'linewidth',2);
xlabel('time (sec)');xlabel('path length (lambda) (m)');

%***********************************
% index path with time (with spline)
%***********************************
% first compute p' in the spline region
ps_prime=diff(p_s')'./diff(l_s);
% find the maximum path velocity that won't violate the velocity constraint
lsdotmax=min(min(abs(umax./ps_prime)')');

% take the slowest time for rotational/positional velocity
% to account for all constaints (using constant 
% rotational/positional speeds)

max_velocity_from_w = abs(lt(end)/((norm(theta1-theta0))/wmax));

% use a single constant velocity for the entire path 

ldotmax=min([l1dotmax l2dotmax lsdotmax max_velocity_from_w]);
% calculate the time needed to travel the complete path
total_time=l4(end)/ldotmax; % faster because the path length is shorter

% find max rotation velocity that wont violate wmax constraint
actual_wmax = (norm(theta1-theta0))/total_time;

% calculate p' for the entire path
p3_prime=diff(pathtotal')'./diff(l4);
% add one more at the end because taking the difference loses one element
p3_prime=[p3_prime p3_prime(:,end)];
% set up time vector at regular sampling period
t3=(0:ts:total_time);NT3=length(t3);
% set up storage space
u3=zeros(2,NT3-1);
pt3=zeros(2,NT3);pt3(:,1)=p0;
w3=zeros(1,NT3);w3(:,1)=actual_wmax;
th3=zeros(1,NT3);th3(:,1)=theta0;
lt3=zeros(size(t3));
pt3prime=zeros(2,NT3);
% the initial path slope is just from the path itself
% this normalizes the velocity vector
pt3prime(:,1)=(pathtotal(:,2)-pathtotal(:,1))/norm(pathtotal(:,2)-pathtotal(:,1));
% use robot kinematics to propagate, and the approximate p' and the maximum
% path velocity to set the robot velocity

for i=1:NT3-1
    u3(:,i)=pt3prime(:,i)*ldotmax;
    pt3(:,i+1)=pt3(:,i)+ts*owmr(pt3(:,i),u3(:,i),umax,-umax);
    w3(:,i+1)=actual_wmax;
    th3(:,i+1)=th3(:,i)+ts*actual_wmax;
    lt3(i+1)=lt3(i)+ts*ldotmax;
    xprime=interp1(l4(1:end),p3_prime(1,:),lt3(i+1));
    yprime=interp1(l4(1:end),p3_prime(2,:),lt3(i+1));
    pt3prime(:,i+1)=[xprime;yprime];
end
% compare the ideal path vs. the approximate path using robot kinematics
figure(13);
plot(lt3,pt3,l4,pathtotal,'x','linewidth',2);
legend('x kinematics','y kinematics','x path','y path');
xlabel('lambda (m)');xlabel('robot position (m)');
title('comparison between robot motion based on kinematics model and geometric path');

% plot spline path
h200=showroom(200,5,5);hold on
smoothn = length(pathtotal);
for i=1:smoothn;robot.Pose(1:2,4)=pathtotal(:,i);show(robot);end

% plot robot orientation on spline path
smootht2=(0:deltatheta/(smoothn-1):deltatheta);
smooththetapath_short = theta0*(1-smootht2/deltatheta) + theta1*smootht2/deltatheta;
smooththetavectors_short = getxy(smooththetapath_short);

h201 = showroom(201,5,5);
q = quiver(pathtotal(1,:),pathtotal(2,:),smooththetavectors_short(1,:),smooththetavectors_short(2,:),0.3);
q.LineWidth = 1.2;
%*****************************************************************************************************************************

figure(14);

yyaxis left
grid on
plot(t3,pt3(1,:),'LineStyle','-','linewidth',2);
hold on
plot(t3,pt3(2,:),'LineStyle','--','linewidth',2);
ylabel('robot position (m)');

yyaxis right
plot(t3,th3,'linewidth',2);
ylabel('robot orientation (rad)');

grid on
legend('x','y','theta','location','best');
xlabel('time (s)')
title('robot motion based on kinematics model');


figure(15);
yyaxis left 
plot(t3(1:end-1),u3(1,:),'linewidth',2);
hold on
plot(t3(1:end-1),u3(2,:),'linewidth',2);legend('u_x','u_y');
ylabel('velocity (m/s)');

yyaxis right
plot(t3(1:end), w3,'linewidth',2)
ylabel('angular velocity (rad)')

grid on
legend('u_x','u_y','w');
xlabel('time (s)');
title('robot velocity');

%
% constant path speed: ldotmax
% path with spline: l4 and path3 
%
% calculate real time from both constraints
t3a=l4/ldotmax; % I feel like something's still wrong here
% calculate speed for each segment
u3a=diff(pathtotal')'./diff(t3a);
w3a=zeros(1,length(u3a)+1)+actual_wmax;
%
figure(24);
yyaxis left
plot(t3a,pathtotal(1,:),'linewidth',2);
hold on
plot(t3a,pathtotal(2,:),'linewidth',2);
ylabel('position (m)')

yyaxis right
plot(t3a,smooththetapath_short,'linewidth',2)
ylabel('orientation (rad)')

grid on
xlabel('time (sec)');
legend('x','y','theta');
title('robot motion based on kinematics model');

figure(25);
yyaxis left
plot(t3a(1:end-1),u3a(1,:),'linewidth',2);
hold on
plot(t3a(1:end-1),u3a(2,:),'linewidth',2);
ylabel('velocity (m/s)')

yyaxis right
plot(t3a(1:end-1),w3a(1:end-1),'linewidth',2)
ylabel('angular velocity (rad/s)')

grid on
xlabel('time (sec)')
legend('u_x','u_y','w');
title('robot velocity')

% use trapezoidal profile
%
% need to figure out bounds on lddot
% 
amax=[.2;.2]; %maximum vehicle acceleration
lddotmax=min(min(abs(amax./ps_prime)')'); 


[x,v,a,ta,tb,tf]=trapgen(0,l4(end),0,0,ldotmax,lddotmax,lddotmax,0);
fprintf('tf for trapezoidal profile = %g\n',tf);
N=100;
t_trap=(0:tf/N:tf);
for i=1:length(t_trap)
  [x(i),v(i),a(i),ta,tb,tf]=trapgen(0,l4(end),0,0,ldotmax,lddotmax,lddotmax,t_trap(i));  
end
% use interpolation to find the time index for each path point on l3
t3b=interp1(x,t_trap,l4);
% calculate speed for each segment
u3b=diff(pathtotal')'./diff(t3b);u3b=[zeros(2,1) u3b(:,1:end-1) zeros(2,1)];
%
figure(34);plot(t3b,pathtotal,'linewidth',2);legend('x','y');
xlabel('time (sec)');xlabel('robot position (m)');
title('robot motion based on kinematics model');

figure(35);

yyaxis left
plot(t3b,u3b(1,:),'linewidth',2);
hold on
plot(t3b,u3b(2,:),'linewidth',2);
ylabel('velocity (m/s)')

yyaxis right
plot(t3b,w3a,'linewidth',2);
ylabel('angular velocity (rad/s)')

grid on
legend('u_x','u_y','w');
xlabel('time (sec)');
title('robot velocity')

%
% set point feedback control
%
tfbk=(0:ts:t3(end)+2);
Nfbk=length(tfbk);
ufbk=zeros(2,Nfbk-1);
w_next = zeros(1,Nfbk-1)+actual_wmax;
Kp = diag([.5,.5]);
pfbk=zeros(2,Nfbk);pfbk(:,1)=p0;
for i=1:Nfbk-1
    ufbk(:,i)= max(min(- Kp * (pfbk(:,i)-p1),umax),-umax);
    pfbk(:,i+1)=pfbk(:,i)+ts*owmr(pfbk(:,i),ufbk(:,i),umax,-umax);
end

figure(54);plot(tfbk,pfbk,'linewidth',2);legend('x','y');
xlabel('time (sec)');xlabel('robot position (m)');
title('robot motion based on setpoint feedback');

figure(55);

yyaxis left
plot(tfbk(1:end-1),ufbk(1,:),'linewidth',2);
hold on
plot(tfbk(1:end-1),ufbk(2,:),'linewidth',2);
ylabel('velocity (m/s)')

yyaxis right
plot(tfbk(1:end-1),w_next,'linewidth',2);
ylabel('angular velocity (rad/s)')

grid on
legend('u_x','u_y','w');
xlabel('time (s)');
title('robot velocity for setpoint feedback control')


tfbka=(0:ts:t3(end)+2);
Nfbka=length(tfbka);
ufbka=zeros(2,Nfbka-1);
Kp = diag([.5,.5]);
pfbka=zeros(2,Nfbka);pfbka(:,1)=p0;
ptarget=zeros(2,Nfbk);

for i=1:Nfbka-1
    if tfbka(i)>t(end)
        ptarget(:,i)=pt(:,end);
    else
        ptarget(:,i)=interp1(t,pt',tfbka(i))';
    end
    ufbka(:,i)= max(min(- Kp * (pfbka(:,i)-ptarget(:,i)),umax),-umax);
    pfbka(:,i+1)=pfbka(:,i)+ts*owmr(pfbka(:,i),ufbka(:,i),umax,-umax);
end

figure(64);plot(tfbka,pfbka,'linewidth',2);legend('x','y');
xlabel('time (sec)');xlabel('robot position (m)');
title('robot motion based on tracking control');

figure(65);plot(tfbka(1:end-1),ufbka,'linewidth',2);legend('u_x','u_y');
xlabel('time (sec)');xlabel('robot position (m)');
title('robot velocity for tracking control')




















%
% show room 
%  
% input:    fignum = figure number
%           xdim = horizontal dimension of room
%           ydim = vertical dimension of room
%
function h=showroom(fignum,xdim,ydim)
    h=figure(fignum);
    hold on
    
    % adjust the view so you can see from above
    view(0,90)
    % adjust the figure range so you can see the 5x5 room
    axis([0 xdim 0 ydim]);
    grid on 
    axis square
    grid minor
    set(gca,'xtick',[0:0.5:5])
    set(gca,'ytick',[0:0.5:5])
    
    zed=.1;
    smolthick = 0.025;

%     robot = collisionCylinder(0.3,zed);
%     robot.Pose(1:2,4)=[1.55;4];
%     robot.Pose(1:2,1:2)=rot2(0);

    w1 = collisionBox(0.05,5,zed);
    w1.Pose(1:2,4)=[0-smolthick;2.5];
    w1.Pose(1:2,1:2)=rot2(0);

    w2 = collisionBox(0.05,5,zed);
    w2.Pose(1:2,4)=[2.5;5+smolthick];
    w2.Pose(1:2,1:2)=rot2(pi/2);

    w3 = collisionBox(0.05,5,zed);
    w3.Pose(1:2,4)=[5+smolthick;2.5];
    w3.Pose(1:2,1:2)=rot2(0);

    w4 = collisionBox(0.05,5,zed);
    w4.Pose(1:2,4)=[2.5;0-smolthick];
    w4.Pose(1:2,1:2)=rot2(pi/2);

    human = collisionCylinder(0.2,zed);
    human.Pose(1:2,4)=[1;0.5];
    human.Pose(1:2,1:2)=rot2(0);

    table = collisionBox(0.5,0.5,zed);
    table.Pose(1:2,4)=[2.5;2.5];
    table.Pose(1:2,1:2)=rot2(pi/4);

    shelf = collisionBox(0.8,0.3,zed);
    shelf.Pose(1:2,4)=[1;4];
    shelf.Pose(1:2,1:2)=rot2(-pi/2);

    % adjust the view so you can see from above

    % adjust the figure range so you can see the 5x5 room
    axis([-2*smolthick 5+2*smolthick -2*smolthick 5+2*smolthick]);

    show(w1)
    show(w2)
    show(w3)
    show(w4)
%     show(robot)
    show(human)
    show(table)
    show(shelf)
end

%
% 2D rotation matrix
% 
% input = theta (angle)
% output = 2x2 SO(2) matrix
%
function R=rot2(theta)
    c=cos(theta);s=sin(theta);
    R=[c -s;s c];
end

%
% kinematics of an omni-directional wheeled mobile robot
%
% input: 
%       X = current state (x, y, theta)
%       U = current input (xdot, ydot, thetadot)
% 
% output: 
%       Xdot = current velocity
%
function Xdot = owmr(X,U,Umax,Umin)
    Xdot = max(min(U,Umax),Umin);
end

%
% kinematics of an nonholonomic wheeled mobile robot
%
% input: 
%       X = current state (x, y, theta)
%       U = current input (u=forward velocity, w=turning angular velocity)
% 
% output: 
%       Xdot = current velocity
%
function Xdot = nwmr(X,U,Umax,Umin)

    Xdot = [cos(X(3)) 0;sin(X(3)) 0; 0 1]*max(min(U,Umax),Umin);
    
end

%
% get unit vector from angle
%
% input:
%       theta = angle
%
% output:
%       x = x-component
%       y = y-component
function [xy] = getxy(theta)
    x = cos(theta);
    y = sin(theta);
    xy = [x;y];
    
end