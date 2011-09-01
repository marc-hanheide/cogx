function plotDaisy(D, xc, yc, color, varargin)
% Plots the daisy descritor figure on a ceartain image position
%
% Syntax:
%   PLOTDAISY(D, xc, yc, color, varargin)
% Parameters:
%   D       Daisy descriptor of the image with it's 200 bins
%   xc      Position
%   yc      Position
%   colour  Plot Color
%   varargin Not yet used
%
% Author: mb (Markus Bader)
% Version: 0.1
% Spetember 2008

    if (nargin<4)
        color='green';
    end
    
    % set default
    sOpt.R = 15;
    sOpt.RQ = 3;
    sOpt.THQ = 8;
    sOpt.HQ = 8;

    alphaStep = 2*pi/sOpt.HQ;
    RingRadii = [2.5 7.5 15]; 
    %Rings
    
    region = 0;
    bin = region * 8 + 1;
    H = D(bin:bin+7); 
    plotGradHis(xc, yc, H, color)
    region = region + 1;
    
    
    for rq = 1:sOpt.RQ
        rira = RingRadii(rq);
        rora = rira/1.5;
        T = -alphaStep/2: alphaStep : 2*pi;
        X = cos(T) * rira + xc;
        Y = sin(T) * rira + yc;
        for i = 1:8
            %plotCircle(x, y, rora, color)
            %plot(x, y, '+', 'Color', 'blue') 
            bin = region * 8 + 1; 
            H = D(bin:bin+7); 
            colorPlot = color;
            if i > 1
                plotGradHis(X(i), Y(i), H, colorPlot, 5);
            else
                plotGradHis(X(i), Y(i), H, colorPlot, 5, 3);
            end
            text(X(i), Y(i),num2str(sum(H),'%1.2f')) 
            region = region + 1;
        end
        % Plot angle arrow
        if rq == sOpt.RQ 
            T = linspace(T(1), T(2), 50);
            X = (cos(T)*(rira+rora)) + xc;
            Y = (sin(T)*(rira+rora)) + yc;
            plot(X,Y,'Color', color);
            plot(X(1),Y(1), 'o', 'Color',color);
        end
    end   
end


function plotGradHis(xc, yc, H, color, size, arrowsize)
    if (nargin<4)
        color='green';
    end
    if (nargin<5)
        size=5;
    end
    hold on
    angleStep = 2*pi/length(H);
    T = 0 : angleStep : 2*pi-angleStep;
    X = (cos(T)*size);
    Y = (sin(T)*size);
    for i = 1:length(H)
        Xl = [xc (X(i)*H(i) + xc)];
        Yl = [yc (Y(i)*H(i) + yc)];
        plotArrow(Xl, Yl, color)
    end
    % Plot angle arrow
    if (nargin>5)
        T = linspace(T(1), T(2), 50);
        X = (cos(T)*arrowsize) + xc;
        Y = (sin(T)*arrowsize) + yc;
        plot(X,Y,'Color', color);
        plot(X(1),Y(1), 'o', 'Color',color);
    end
end

function plotCircle(xc, yc, r, color)
    if (nargin<4)
        color='green';
    end
    hold on
    theta = linspace(0,2*pi,50);
    x = cos(theta)*r+xc;
    y = sin(theta)*r+yc;  
    plot(x,y,'Color', color);
end


function plotArrow(X, Y, color)
    if (nargin<3)
        color='green';
    end
    line(X,Y,'Color', color);
end