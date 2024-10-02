% Clearing the screen
clear();

%defining required variables
%for DH table
syms alpha a d theta;
%for angles
syms th1 th2 th3 th4 th5 th6;
%for a and d
% syms a1 a2 d2 d d6;
syms x y z;

% Given
a1 = 1;
a2 = 1;
th1 = pi/3;
th2 = pi/6;
DHTable1 = [0 0 0 th1; 
            0 a1 0 th2;]; %Giving DH Table
Anen = [1; 0; 0;];
DHTable = subs(DHTable1);

%Calling function
[T0toN, PE0,Px, Py,Pz ] = calcPos(DHTable, Anen);

Rmat = T0toN(1:3,1:3);
Omat = T0toN(1:3,4);

disp("The Transformation Matrix is:");
disp(T0toN)

disp("The Rotation Matrix is:");
disp(Rmat)

disp("The Translation Matrix is:");
disp(Omat)

disp("The final postion vector is:");
disp(PE0)

%Ploting Links
figure(1)
% contour3(P);
plot3(Px,Py,Pz,'-o');
xlabel("x-axis");
ylabel("y-axis");
zlabel("z-axis");
legend("Theta1 = 60Deg theta2 = 30Ddeg")
title("Qustion 1");
%------------------------------------------------------------------
% Question 2
%------------------------------------------------------------------
% Given
a2 = 0.5;
d2 = 0.5;
d4 = 1;
d6 = 1;
th1 = 0;
th2 = pi/2;
th3 = -pi/2;
th4 = 0;
th5 = 0;
th6 = 0;
DHTable2 = [0      0     0    th1;
            pi/2   0  (-d2)   th2;
            0      a2   0     th3;
            (-pi/2) 0   d4    th4;
            pi/2    0   0     th5;
            (-pi/2) 0   0     th6;]; %Giving DH Table
DHTable = subs(DHTable2);
Anen = [0; 0; d6;]
%Calling Function
[T0toN, PE0,Px, Py,Pz ] = calcPos(DHTable, Anen);

%extractating Rotation and Translation Matrix from Transformation matrix 
Rmat = T0toN(1:3,1:3);
Omat = T0toN(1:3,4);

disp("The Transformation Matrix is:");
disp(T0toN)

disp("The Rotation Matrix is:");
disp(Rmat)

disp("The Translation Matrix is:");
disp(Omat)

disp("The final postion vector is:");
disp(PE0) 

%Ploting links
figure(2)
% contour3(P);
plot3(Px,Py,Pz,'-o');
xlabel("x-axis");
ylabel("y-axis");
zlabel("z-axis");
title("Qustion 2");
%-------------------------------------------------
% Function take DH-Table and Anen as a Argument and return transformation matrix , Position matrix and list of x,y and z coordinates of the respective Links
function  [T0toN , PE0, Px, Py,Pz ,P] = calcPos(DHTable, Anen)
    T0toN = eye(4,4);
    s = size(DHTable);
    s = s(1);
    Px = [0];
    Py = [0];
    Pz = [0];
    P = [0,0,0];
    for i = 1:s
        alpha = DHTable(i,1);
        a = DHTable(i,2);
        d = DHTable(i,3);
        theta = DHTable(i,4);
        Tmati = [ cos(theta) -sin(theta) 0 a;
    sin(theta)*cos(alpha) cos(theta)*cos(alpha) -sin(alpha) -d*sin(alpha); 
    sin(theta)*sin(alpha) cos(theta)*sin(alpha) cos(alpha) d*cos(alpha);
    0 0 0 1;];
        T0toN = T0toN*Tmati;
        pxy =   T0toN*[0;0;0;1]
        Px = [Px pxy(1)]
        Py = [Py pxy(2)]
        Pz = [Pz pxy(3)]
%         P = [P; pxy(1) pxy(2) pxy(3)]
    end
    PE0And1Mat = T0toN*[Anen;1];
    PE0 = PE0And1Mat(1:3,1);
    Px = [Px PE0(1)]
    Py = [Py PE0(2)]
    Pz = [Pz PE0(3)]
end
