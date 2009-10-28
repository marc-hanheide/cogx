%%
% Originally a part of: curiousDanijel (developed within EU project CogX)
% Author: Matej Kristan, 2009 (matej.kristan@fri.uni-lj.si; http://vicos.fri.uni-lj.si/matejk/)
% Last revised: 2009
%%
function showSurfaceFromPoints( x, rgb3d, LRaxRoi )

turnoffDelaunay = 1 ;

if nargin < 2
    rgb3d = [] ;
end

if nargin < 3
    LRaxRoi = [] ;
end


x(:,3) = -x(:,3) ;
if turnoffDelaunay == 0
    TRI = delaunay(x(:,1),x(:,2));
else
    TRI = [] ;
end
if isempty(rgb3d)
    if ~isempty( LRaxRoi )
        trisurf(TRI,x(:,1),x(:,2),x(:,3),'Parent',LRaxRoi) ;
    else
       trisurf(TRI,x(:,1),x(:,2),x(:,3)) ;  
    end
 
    colormap bone ;
else
     if isempty( LRaxRoi )
        trisurf(TRI,x(:,1),x(:,2),x(:,3),[1:size(x(:,3),1)]') ;
    else
       trisurf(TRI,x(:,1),x(:,2),x(:,3),[1:size(x(:,3),1)]','Parent',LRaxRoi) ;
    end
    
    colormap(rgb3d/255) ;
%     shading interp ;
end
hold on ;
if ~isempty( LRaxRoi )
    plot3(x(:,1),x(:,2),x(:,3),'r.','Parent',LRaxRoi) ;
else
    plot3(x(:,1),x(:,2),x(:,3),'r.') ;
end
axis equal ; axis tight ; grid off ;

view([37, 42]) ; box on ; 
set(gca,'XTick',[]) 
set(gca,'YTick',[]) 
set(gca,'ZTick',[]) 

