%clearing the previous screens
clear;
clear all;
clc();

%disable waringing to be shown in output
warning('off','all');

%defining general variables
syms th1 th2 th3 th4 th5 th6;
syms a1 a2 a3 a4 a5 a6;
syms d1 d2 d3 d4 d5 d6;
syms alp1 alp2 alp3 alp4 alp5 alp6;
syms l1 l2;

% Given
a2 = 0.5;
d1 = 0.5;
d4 = 1;
d6 = 1;

%DH table of experiment 3 of 6-link 
DHTable3 = [0      0     0    th1;
            pi/2   0  (-d1)   th2;
            0      a2   0     th3;
            (-pi/2) 0   d4    th4;
            pi/2    0   0     th5;
            (-pi/2) 0   0     th6;]; %Giving DH Table

%DH Table of 4 link
DHTable2 = [0      0     0    th1;
            0      l1    0   th2;
            pi      l2   d3    0;
            0        0   0     th4;]; %Giving DH Table

%DH Table of 2 link
DHTable1 = [ 0      0    0     th1;
            -pi/2  a1   d2      0;];


Anen1 = [0;0;1;]; %End factor of 2 link
Anen2 = [0;0;0;]; %end factor of 4-link


% For 2-link Robot
DHTable = subs(DHTable1);
var = VariableList(DHTable);
J = jacobian(DHTable,Anen1,var);
disp("Dh Table of 2-link robot= ")
disp(DHTable)
disp("Jacobian Matrix is :")
J = simplify(J);
disp(J)
calcSingularities(J,var);


%-------------------------------------------------
% For 4-link Robot

DHTable = subs(DHTable2);
var = VariableList(DHTable);
J = jacobian(DHTable,Anen2,var);
disp("Dh Table of 4 link robot= ");
disp(DHTable);
disp("jacobian Matrix is :");
J = simplify(J);
disp(J);
calcSingularities(J,var);
    

%-------------------------------------------------

%Function to print singularities of the Jacobian
function calcSingularities(J,var) 
    
    Jtrans = transpose(J);
    equSingularity = det(sqrt(Jtrans*J));
    expSing = simplify(equSingularity);
    disp("Singularity equation: ");
    disp(expSing);
    for i=1:length(var)
        
        if has(expSing,var(i))
            sol = (solve(expSing==0,var(i), real=true));
            exp = sol(sol==real(sol));
            disp(exp)
            if (length(exp) == 0)
                disp("For singularity, No real values exsist of");
                disp(var(i));
            else 
                disp("Singularity for vaule of ")
                disp(var(i))
                disp(exp);
            end
        else
            disp("Singularity does not depend on value of")
            disp(var(i))
        end
    end
end

%Function to calculate jacobian of the Given DHtable
function  J = jacobian(DHTable,Anen,var)
    
    
    E0toN = caclE0toN(DHTable,Anen);
    
    jv = [];
    jw = [];
    
    T0toN = eye(4,4);
    s = size(DHTable);
    s = s(1);
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

        if var(i,2)==3 % joint is prismatic
            e = 0;
        end
        if var(i,2)==4 %  joint is revolute
            e = 1;
        end

        O0toi = T0toN(1:3,4);
        Z0toi = T0toN(1:3,3);

        ji = cross((e*Z0toi),(E0toN - O0toi)) + (1-e)*Z0toi ;
        jj = e*Z0toi;

        jv = [jv ji];
        jw = [jw jj];
        
    end
    J  = [jv;jw;];
end

%Function to calculate E0toN value i.e. end factor  with respect to the 
function  E0toN = caclE0toN(DHTable, Anen)
    T0toN = eye(4,4);
    s = size(DHTable);
    s = s(1);
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

    end
    PE0And1Mat = T0toN*[Anen;1];
    E0toN = PE0And1Mat(1:3,1)
end

%function will return the list of variaables along with they are revolute joint or prismatic joint
function var = VariableList(DhTable)
    s = size(DhTable);
    var = [];
    for i=1:s(1)
        for j=3:4
            if not(isSymType(DhTable(i,j),'constant'))
                var1 = [DhTable(i,j), j];
                var = [var; var1];
            end
        end
    end
end