
px = 1;
py = 1;
ph = 20*pi/180;
radius = 0.4;
centre = [0,1];

plotRobot(centre,radius,ph)

%function will return theta 1, theta2, and theta3
%All angles are in Radian
function [th1pos,th1neg,th2pos,th2neg,th3pos,th3neg] = invRobot(px,py,phi)
    %defining robot constans
    a1 = 1;
    a2 = 1;
    a3 = 0.5;

    wx = px - (a3 * cos(phi));
    wy = py - (a3 * sin(phi));
    
    D = (wx*wx + wy*wy - a1*a1 - a2*a2)/(2*a1*a2);

    th2pos = atan2((sqrt(1-D*D)),D);
    th2neg = atan2((-sqrt(1-D*D)),D);
    th1pos = atan2(wy,wx) - atan2((a2*sin(th2pos)),(a1+(a2*cos(th2pos))));
    th1neg = atan2(wy,wx) - atan2((a2*sin(th2neg)),(a1+(a2*cos(th2neg))));
    th3pos = phi - th1pos - th2pos;
    th3neg = phi - th1neg - th2neg;
end

function plotRobot(centre,r,ph)
    %defining robot constans
    cx = centre(1);
    cy = centre(2);
    a1 = 1;
    a2 = 1;
    a3 = 0.5;
    
    phi = linspace(0,2*pi,100);

    traceX = [];
    traceY = [];

    traceXneg = [];
    traceYneg = [];
    th1graphs=[];
    th2graphs=[];
    th3graphs=[];
    th1graphsNeg=[];
    th2graphsNeg=[];
    th3graphsNeg=[];
    t = [];

    for i = 1:length(phi)
        t = [t i/100]

        rx = cx + r * cos(phi(i));
        ry = cy + r * sin(phi(i));
        
        [th1,th1neg,th2,th2neg,th3,th3neg] = invRobot(rx,ry,ph);
        th1graphs = [th1graphs th1];
        th2graphs = [th2graphs th2];
        th3graphs = [th3graphs th3];
        

        x1 = a1 * cos(th1) ;
        y1 = a1 * sin(th1) ;
    
        wx = x1 + a2*cos(th1 + th2);
        wy = y1 + a2*sin(th1 + th2);
    
        px = wx + a3*cos(th1 + th2 + th3);
        py = wy + a3*sin(th1 + th2 + th3);
        
        linksX = [0 x1 wx px];
        linksY = [0 y1 wy py];
        
        traceX = [traceX px];
        traceY = [traceY py];
        figure(1)
       
        subplot(2,2,1);
        
        plot(linksX, linksY,traceX,traceY);
        axis([-3 3 -3 3]);%fixing axis plane
        axis square
      
        

        %--------------------------------------------
        %ploting for negative
        th1 = th1neg;
        th2 = th2neg;
        th3 = th3neg;

        th1graphsNeg = [th1graphsNeg th1];
        th2graphsNeg = [th2graphsNeg th2];
        th3graphsNeg = [th3graphsNeg th3];
    
        x1neg = a1 * cos(th1);
        y1neg = a1 * sin(th1);

        wxneg = x1neg + a2*cos(th1 + th2);
        wyneg = y1neg + a2*sin(th1 + th2);
    
        pxneg = wxneg + a3*cos(th1 + th2 + th3);
        pyneg = wyneg + a3*sin(th1 + th2 + th3);
        
        linksXneg = [0 x1neg wxneg pxneg];
        linksYneg = [0 y1neg wyneg pyneg];
        
        traceXneg = [traceXneg pxneg];
        traceYneg = [traceYneg pyneg];
        subplot(2,2,2);
        
        plot(linksXneg, linksYneg,traceXneg,traceYneg);
        axis([-3 3 -3 3]);%fixing axis plane
        axis square

        pause(0.03)

        
        subplot(2,2,3);        
        plot(t,th1graphs,t,th2graphs,t,th3graphs)
        legend('theta1',"theta2","theta3")
        xlabel('time')
        axis([0 1.5 -7 7])

        subplot(2,2,4);        
        plot(t,th1graphsNeg,t,th2graphsNeg,t,th3graphsNeg)
        legend('theta1',"theta2","theta3")
        xlabel('time')
        axis([0 1.5 -7 7])
    
    end

    

%         x1 = a1 * cos(th1);
%         y1 = a1 * sin(th1);
%     
%         wx = x1 + a2*cos(th1 + th2);
%         wy = y1 + a2*sin(th1 + th2);
%     
%         px = wx + a3*cos(th1 + th2 + th3);
%         py = wy + a3*sin(th1 + th2 + th3);
%     
%         linksX = [0 x1 wx px]
%         linksY = [0 y1 wy py]
%         figure(1)
%         plot(linksX, linksY);
%         axis([-3 3 -3 3]);%fixing axis plane
end





