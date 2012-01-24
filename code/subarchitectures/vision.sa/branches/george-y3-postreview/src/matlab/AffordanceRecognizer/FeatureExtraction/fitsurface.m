function [solution eigs sumsquarederror] = fitsurface(P, varargin)

    visualise_flag = false;

    switch nargin
        case 2
            visualise_flag = varargin{1};
    end

    %load FGVTXPepsican.mat
    %load FGVTXPhone.mat

    %P = FGVTXBluecube;
    %P = FGVTXLadybird;
    %P = FGVTXPepsican;
    %P = FGVTXPhone;

    % let us assume that these points can be fitted by 

    % z=1/2ax2+bxy+1/2cy2+dx+ey+f

    x=P(:,1);
    y=P(:,2);
    z=P(:,3);

    % .. and so the eqs to be solved by least squares are .. 

    lhs=[1/2*x.^2, x.*y, 1/2*y.^2, x , y, ones(size(x))];
    % lhs=[x.^2, x.*y, y.^2, x , y, ones(size(x))];
    % lhs=[1/2*x.^2, x.*y, 1/2*y.^2];
    lhs=[1/2*x.^2, x.*y, 1/2*y.^2, x , y, ones(size(x))];
    % lhs=[x.^2, x.*y, y.^2, x , y, ones(size(x))];
    % lhs=[1/2*x.^2, x.*y, 1/2*y.^2];
    rhs=z; 
    solution=lhs\rhs; % a b c d e f
    
    sumsquarederror = mean((lhs*solution - rhs).^2);

    % show the surface plus the points 
    xmin=min(x); 
    xmax=max(x); 
    xsteps=(xmax-xmin)/100; 
    ymin=min(y); 
    ymax=max(y); 
    ysteps=(ymax-ymin)/100; 

    [X,Y]=meshgrid(xmin:xsteps:xmax, ymin:ysteps:ymax); 
    sz=size(X); 
    X=X(:); Y=Y(:);
    Z=[1/2*X.^2, X.*Y, 1/2*Y.^2, X , Y, ones(size(X))]*solution;
    % Z=[X.^2, X.*Y, Y.^2, X , Y, ones(size(X))]*solution;
    % Z=[1/2*x.^2, x.*y, 1/2*y.^2]*solution;
    X=reshape(X,sz); 
    Y=reshape(Y,sz); 
    Z=reshape(Z,sz);
    
    if visualise_flag
        h=surf(X,Y,Z); set(h,'lines','n')
        hold on
        plot3(x,y,z,'g*');
        hold off
        axis equal
    end

    a=solution(1); 
    b=solution(2); 
    c=solution(3);

    mat=[a b 
         b c]; 

    [eigs]=eig(mat);