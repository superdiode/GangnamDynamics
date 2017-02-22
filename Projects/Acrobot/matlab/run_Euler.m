function run_Euler()
clear all;
clc;
close all;

FinalTime = 30;         % sec
dt = 0.001;             % time step (sec)
t = 0:dt:FinalTime;     % time
n = length(t);

obj = AcrobotPlant;
obj.l1 = 10; 
obj.l2 = 10;  
obj.m1 = 1; 
obj.m2 = 1;  
obj.lc1 = 5; 
obj.lc2 = 5;

q = [pi/2;pi/2];
q_dot = [0;0];
q_dotdot = [0;0];

y = zeros(4,1);

for i=1 : 1 : n-1 

u = [0;0];
[H,C,B] = manipulatorDynamics(obj,q,q_dot);
q_dotdot = inv(H)*(u - C);

q_dot = q_dot + dt*q_dotdot;
q = q + dt*q_dot;

qData(:,i) = q;

end


%% animation

figure(2)
Ax = [0,0]; Ay = [0,0];
title('Double pendulum v1');
axis([-25 25 -25 25])
xlabel('length(m)')
ylabel('length(m)')

lower_link = animatedline;
lower_link.Color = 'magenta';
lower_link.LineWidth = 3;
lower_link.AlignVertexCenters = 'on';
lower_link.Marker = 'o';

upper_link = animatedline;
upper_link.Color = 'blue';
upper_link.LineWidth = 3;
upper_link.AlignVertexCenters = 'on';
upper_link.Marker = 'o';

grid on;

l1 = obj.l1;
l2 = obj.l2;

samplingTime = 100;

for i = 1 : samplingTime : n-1

    theta1 = qData(1,i);
    theta2 = qData(2,i);

    x1 = l1*sin(theta1); % x1 = l1*cos(theta1);
    y1 = -l1*cos(theta1); % y1 = l1*sin(theta1);

    x2 = x1+l2*sin(theta1+theta2); % x2 = x1+l2*cos(theta1+theta2);
    y2 = y1-l2*cos(theta1+theta2); % y2 = y1+l2*sin(theta1+theta2);

    % ========= LOWER LINK ==========    
    Ax = [0,x1];
    Ay = [0,y1];

    clearpoints(lower_link)
    addpoints(lower_link,Ax,Ay);

    % ========= UPPER LINK ==========  
    Ax = [x1,x2];
    Ay = [y1,y2];

    clearpoints(upper_link)
    addpoints(upper_link,Ax,Ay);

    drawnow
end

end