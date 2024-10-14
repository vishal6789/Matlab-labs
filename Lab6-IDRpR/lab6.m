clc();
clear;
clear all;

q1_0 = 0;
q1_T = 2*pi/3;
q2_0 = 0.5;
q2_T = 1;
T = 3;
m2 = 1.5;
l2 = 1.5;
g = 9.81;


t = 0:0.05:T;

an = (2*pi*t)/T;
c1 = (q1_T - q1_0)/T;
c2 = (q2_T - q2_0)/T;

%----------------------------------------------------------
%part A - a
%q1 and q2 desired

q1_des = q1_0 + c1*(t - (T/(2*pi))*sin(an) );

q1dot_des = c1*(1-cos(an));

q1ddot_des = c1*((2*pi/T)*sin(an));

%q2 des
q2_des = q2_0 + c2*(t - (T/(2*pi))*sin(an) );

q2dot_des = c2*(1-cos(an));

q2ddot_des = c2*((2*pi/T)*sin(an));


%plotting q1
figure(1)
plot(t,q1_des, t, q1dot_des, t, q1ddot_des)
xlabel('Time(sec)')
legend('theta1', 'theta1 dot' , 'theta1 double dot');
title("Desired q1 w.r.t Time")

%plotting q2
figure(2);
plot(t,q2_des, t, q2dot_des, t, q2ddot_des);
xlabel('Time(sec)');
legend('d2', 'd2 dot' , 'd2 double dot');
title("Desired q2 w.r.t Time");

%----------------------------------------------------------
%part A - b
%finding Torque and force
torque = [];
force2 = [];

s = size(t);
s = s(1,2);

for i = 1:s
    I = [
        m2*(q2_des(i)^2)    0;
            0               m2;
    ];
    
    qddot = [
        q1ddot_des(i);
        q2ddot_des(i);
    ];

    C = [
        m2*q2_des(i)*q2dot_des(i)         m2*q2_des(i)*q1dot_des(i);
        -m2*q2_des(i)*q1dot_des(i)  0;
    ];

    qdot = [
        q1dot_des(i);
        q2dot_des(i);
    ];

    h = [
        m2*q2_des(i)*sin(q1_des(i))*g;
        -m2*cos(q1_des(i))*g;
    ];


    solMat = I*qddot + C*qdot + h;
    torque = [torque solMat(1,:)];
    force2 = [force2 solMat(2,:)];
    
end

%plotting torque and force
figure(3);
plot(t,torque, t, force2);
xlabel('Time(sec)');
legend('Torque Tau1', 'Force f2');
title("Torque and Force w.r.t Time");

%------------------------------------------------------
%Part A - c 
%Animation

xtrace = [];
ytrace = [];
for i = 1:s
    x_point = q2_des(i)*cos(q1_des(i));
    y_point = q2_des(i)*sin(q1_des(i));
    x_points=[0,x_point];
    y_points = [0,y_point];
    xtrace = [xtrace x_point];
    ytrace = [ytrace y_point];
    figure(4)
    plot(x_points,y_points,xtrace,ytrace)
    hold on;
    x_Spoint = (1.5 - q2_des(i))*cos(pi + q1_des(i));
    y_Spoint = (1.5 - q2_des(i))*sin(pi + q1_des(i));
    x_Spoints=[0,x_Spoint];
    y_Spoints = [0,y_Spoint];
    plot(x_Spoints,y_Spoints,color ='#0072BD')
    plot(0,0,'o');
    axis([-1.5 1.5 -1.5 1.5]);
    hold off;
    pause(0.1)
end 