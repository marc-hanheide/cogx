function mxGuiFindCorr(img1, img2, d1, d2, N, R)
% GUIFINDCORR offers a graphical user interface for finding
% correspondences
%
% Syntax:
%   MXGUIFINDCORR(IMG1, IMG2, D1, D2)
%   MXGUIFINDCORR(IMG1, IMG2, D1, D2, N)
%
% Parameters:
%   IMG1    Path to image file or image
%   IMG2    Path to image file or image
%   D1      Dense descriptor for IMG1 (Size: L x W x H)
%   D2      Dense descriptor for IMG2 (Size: L x W x H)
%   N       Number of maches to show
%   R       Radii of the zoomed image
%
% Author: pb (Philipp Blauensteiner)
% Version: 0.1
% April 2008
% Author: mb (Markus Bader)
% Version: 0.2
% August 2008
    

    if (nargin<6)
        R=30;
    end

    if (nargin<5)
        N = 1;
    end
    
    if (nargin<4)
        error('Please mind the usage.');
    end
    
    if (ischar(img1))
        img1 = imread(img1);
    end
    
    if (ischar(img2))
        img2 = imread(img2);
    end
    
    
    fig = figure;
    figWidth = size(img1,2)+size(img2,2);
    figHeight = size(img1,1)*2;
    set(fig,'Position',[1 1 figWidth figHeight]);
    
    subplot(2,2,1);
    p1 = imshow(img1);

    subplot(2,2,2);
    p2 = imshow(img2);

    set( p1, 'ButtonDownFcn', { @ipButtonDown_Callback, img1, img2, d1,d2, fig, 1, N, R} );
    set( p2, 'ButtonDownFcn', { @ipButtonDown_Callback, img2, img1, d2,d1, fig, 2, N, R} );

end

function ipButtonDown_Callback(hObject, eventdata,img1, img2, d1,d2, fig, clicked_subplot, N, R)

   colors = {'yellow', 'magenta', 'cyan', 'red', 'green', 'blue', 'white', 'black' };
            
    cursorPos = get( gca, 'CurrentPoint' );
    xc = round( cursorPos(1,1) );
    yc = round( cursorPos(1,2) );

    if (clicked_subplot==1)
        other_subplot=2; 
    else
        other_subplot=1; 
    end;    
    
    subplot(2,2,clicked_subplot);
    p1 = imshow(img1);
    set( p1, 'ButtonDownFcn', { @ipButtonDown_Callback, img1, img2, d1,d2, fig, clicked_subplot,N, R} );

    da = d1(:,xc,yc);
    
    featCount = 1;
    
    plotFeature(xc,yc,colors{1});
    
    
    
    subplot(2,2,other_subplot);
    hold on;
    p2 = imshow(img2);
    
   
   [distMap, minDist, mc, mr] = eucDist(d2,da);
   db = d1(:,mc,mr);
   plotFeature(mc,mr,colors{1});
   fprintf(1,'<%d %d> ~ <%d %d>\n', xc, yc, mc, mr);
   
   if N > 1
       [sv si]  = sort(distMap(:,1));
       distMap = distMap(si,:);
       for i = 2:N
           plotFeature(distMap(i,2), distMap(i,3), colors{1});
       end
   end
    
    subplot(2,2,other_subplot+2);
    plotSubImage(img2, mc, mr, R, db);
    subplot(2,2,clicked_subplot+2);
    plotSubImage(img1, xc, yc, R, da)
    
    set( p2, 'ButtonDownFcn', { @ipButtonDown_Callback, img2, img1, d2,d1, fig, other_subplot, N, R} );
    
end

function plotSubImage(I, x, y, R , D)
    img2crop = imcrop(I,[x-R y-R R*2 R*2]);
    imshow(img2crop);
    hold on;
    if  nargin < 5
        plotFeature(R+1,R+1,'green')
    else
        plotDaisy(D, R+1, R+1)
    end
end


function plotFeature(x,y,color)
    hold on
    h = plot(x,y,'+','Color', color);
    plotCircle(x, y, 2.5, color)    
    plotCircle(x, y, 7.5, color)    
    plotCircle(x, y, 15,  color)
end


function plotCircle(xc,yc,r, color)
    if (nargin<4)
        color='green';
    end
    hold on
    theta = linspace(0,2*pi,50);
    x = cos(theta)*r+xc;
    y = sin(theta)*r+yc;  
    plot(x, y, 'Color', color);
end


