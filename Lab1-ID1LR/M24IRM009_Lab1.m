% Clearing the screen
clear();

% Declaring required Constants
theta_0 = 0;
theta_T = 2*pi/3;
T = 3;
l = 1;
m = 1;
lc = 0.5;
Izz = 1/3*m*l^2;
g = 9.8;


%% Section 1
% Desired Theta Graphs

t = 0:0.1:T; % Taking Time Intervals
c = 2*pi/T;
x = ((theta_T-theta_0)/T)
y = t-((1/c).*sin(2*pi*t/T))
z = 1- cos(2*pi/T*t);
w = (2*pi/T)*sin(2*pi/T*t);

Theta_des = theta_0 + x.*y;

Theta_des1 = x.*z;

Theta_des2 = x.*w;

figure(1)
plot(t,Theta_des, t, Theta_des1, t, Theta_des2)
xlabel('Time(sec)')
ylabel('Theta')
legend('theta des', 'theta dot 1' , 'theta dot 2');
title("Desired Theta w.r.t Time")

%% Section2
% Velocity and Acceleration Graph
xe = l*cos(Theta_des)
ye = l*sin(Theta_des)
% plot(t,xe,t,ye)
 
xe_dot = -l*sin(Theta_des).*Theta_des1;
ye_dot = l*cos(Theta_des).*Theta_des1;
 
xe_ddot = -l*sin(Theta_des).*Theta_des2 - l*cos(Theta_des).*(Theta_des1.^2);
ye_ddot = l*cos(Theta_des).*Theta_des2 - l*sin(Theta_des).*(Theta_des1.^2);

figure(2)
plot(t,xe_dot,t,ye_dot,t,xe_ddot,t,ye_ddot);
legend('Velocity x-component','Velocity y-component','Accleration x-component','Acceleration y-component');
title("Acceleration and Velocity")
% grid on;



%% Section3
tau = Izz*Theta_des2 + (m*g*lc*cos(Theta_des))

figure(3)
plot(t,tau)
ylabel('Torque')
xlabel('Time')
legend('Torque')
title("Torque w.r.t Time")

%% Section 4
% Animation

% S will contain no of intervals in t(time)
s = size(t) 
s = s(1,2)

% Declaring empty arrays to store  points for tracing
xtrace = []
ytrace = []

% Loop for animation
for i = 1:s
    x_point = xe(i);
    y_point = ye(i);
    x_points=[0,x_point];
    y_points = [0,y_point];
    xtrace = [xtrace x_point]
    ytrace = [ytrace y_point]
    figure(4)
    plot(x_points,y_points,xtrace,ytrace)
    axis([-1.5 1.5 -1.5 1.5]);
    pause(0.1)
end    
