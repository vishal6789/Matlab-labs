%clearing all
clear();
clc();

%defining variables
tau = 0;
thetaT0 = [0 pi/3 pi/3];
dotThetaT0 = [0 0 0.1];
initalThetaStr = ["Theta(0)=0" "Theta(0)=Pi/3" "Theta(0)=Pi/3"];
initialVelStr = [" ThetaDot(0)=0rad/s" " ThetaDot(0)=0rad/s" " ThetaDot(0)=0.1rad/s"];
T = 3;
m = 1;
lc = 0.5;
l = 1;
Izz = (1/3)*(m*l^2);
g  = 9.81;
tSpan = [0 T];



%Initalising loop to solve for all three initial values
for j =1:3
initalValue = [thetaT0(j) dotThetaT0(j)]; %Creating intial values matrix

%solving differtial equation using 
[t ,theta] = ode45(@(t,theta) odefcn(t,theta,m,g,lc,Izz,tau),tSpan, initalValue);
thetaAngle = theta(:,1);
dotTheta = theta(:,2);


%Ploting Theta, Angular velocity, Angular Acceleration
dotDotTheta = (1/Izz)*(tau - (m*g*lc*cos(thetaAngle)));
figure(1)
plot(t,thetaAngle,t,dotTheta,t,dotDotTheta);
xlabel("Time in seconds");
ylabel(initalThetaStr(j)+initialVelStr(j));
legend("Theta", "Angular Velocity", "Angular Acceleration");



% %question 3: Animation
sizee = size(t)
for i = 1:size(t)
   x = l*cos(thetaAngle(i)); %calculating x-cordinate
   y = l*sin(thetaAngle(i)); %calculating y-cordinate
   x_points = [0 x];         %creating x-vector
   y_points = [0 y];         %creating y-vector
   figure(2)
   title("Animation")
   plot(x_points,y_points);
   xlabel(initalThetaStr(j)+initialVelStr(j))
   axis([-1.5 1.5 -1.5 1.5]);%fixing axis plane
   pause(0.1)
end

end

%------------------------------------------------------------------------
initalValue = [thetaT0(1) dotThetaT0(1)];
[t ,theta1] = ode45(@(t,theta1) odefcn1(t,theta1,m,g,lc,Izz),tSpan, initalValue);
% figure(5)
% plot(t,theta1)
thetaAngle1 = theta1(:,1);
xtrace = [];
ytrace = [];

for i = 1:size(t)
   x = l*cos(thetaAngle1(i)); %calculating x-cordinate
   y = l*sin(thetaAngle1(i)); %calculating y-cordinate
   x_points = [0 x];         %creating x-vector
   y_points = [0 y];         %creating y-vector
   figure(3)
   %tracing x and y cordinates
   xtrace = [xtrace x]; 
   ytrace = [ytrace y];
   plot(x_points,y_points,xtrace,ytrace);
   axis([-1.5 1.5 -1.5 1.5]);%fixing axis plane
   pause(0.1)
end
%---------------------------------------------------------
%Defining required functions
function dtheta_dt = odefcn(t,theta,m,g,lc,Izz,tau)
   dtheta_dt = zeros(2,1);
   dtheta_dt(1) = theta(2);
   dtheta_dt(2) = (1/Izz)*(tau - (m*g*lc*cos(theta(1))));
end

function dtheta_dt1 = odefcn1(t,theta1,m,g,lc,Izz)
   dtheta_dt1 = zeros(2,1);
   dtheta_dt1(1) = theta1(2);
   tauInstant = calTau(t);
   dtheta_dt1(2) = (1/Izz)*(tauInstant - (m*g*lc*cos(theta1(1))));
end

%Give Torque at time t according to lab 1 question
function tauInstant = calTau(t)
   theta_0 = 0;
   theta_T = 2*pi/3;
   T = 3;
   l = 1;
   m = 1;
   lc = 0.5;
   Izz = 1/3*m*l^2;
   g = 9.81;
   temp = ((theta_T-theta_0)/T);
    thetaDes  = theta_0 + (temp*(t-((T/(2*pi)*sin(2*pi*t/T)))));
    thetaDotDotDes = temp*((2*pi/T)*sin(2*pi*t/T));
   tauInstant = (Izz*thetaDotDotDes + (m*g*lc*cos(thetaDes)));
end



