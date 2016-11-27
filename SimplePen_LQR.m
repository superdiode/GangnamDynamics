%%
clc; close all; clear

% System Paramters
m = 0.3; b = 0.1; l = 0.5; g=9.81; u=1;

% Simplification
r = g/l;
k = b/(m*l^2); % k = b/(m*l);
p = 1/(m*l^2);

% The Pendulum ODE

% Initial Condition
x = [pi*0.7 0]; % x = [pi 0]+0.2*randn(1,2);


% Linearize the system
dt = 0.01;
kp = 1.8; 

FinalTime  = 1000;

x_arr = zeros(FinalTime,2);
x_arr(1,:) = x;


for i=1:1:FinalTime-1  

    u = kp*(pi - x_arr(i,1));
    f = [x_arr(i,2); p*u - k*x_arr(i,2) - r*sin(x_arr(i,1))];
    x_arr(i+1,:) = x_arr(i,:) + dt*f';

end

x_arr(FinalTime,:)

O = [0 0];

figure(1)
plot(x_arr(:,1),x_arr(:,2),'-r')

%% Loop for animation

% subplot(1,2,2)
% axis(gca, 'equal')
% plot(x_arr(:,1),x_arr(:,2),'-r')
% hold on
% 
% title('phase portrait','fontsize',12)
% xlabel('\theta','fontsize',12)
% ylabel('\thetadot','fontsize',12)

figure(2)

for i=1:1:FinalTime

%     % draw pendulum    
    subplot(1,2,1)

    %     % title
    text(1, 1.3,'\bf LQR control for simple pendulum','HorizontalAlignment','center','VerticalAlignment', 'top')

    title('simple pendulum','fontsize',12)
    xlabel('x','fontsize',12)
    ylabel('y','fontsize',12)
	axis(gca, 'equal') % Aspect ratio of the plot
    axis([-0.7 0.7 -0.7 0.7 ]) % The limit of plot

%     % Mass point
    P = l*[sin(x_arr(i,1)) -cos(x_arr(i,1))];
    
%     % Circle in origin
    O_circ = viscircles(O, 0.01);
    
    
%     % Pendulum
    pend = line([O(1) P(1)],[O(2) P(2)]);
    
%     % Ball
    ball = viscircles(P, 0.05);
    
%     % Time interval to update plot
    pause(0.0001);
   
%     % Delete previous object if it is not the final loop
    if i < FinalTime
        delete(pend);
        delete(ball);
        delete(O_circ);
    end
    
%     % Phase Portrait
    subplot(1,2,2)
%     axis(gca, 'equal')
    % axis([-10 10 -10 10])
    plot(x_arr(i,1),x_arr(i,2),'or')
    hold on
    
    title('phase portrait','fontsize',12)
    xlabel('\theta','fontsize',12)
    ylabel('\thetadot','fontsize',12)

end
 