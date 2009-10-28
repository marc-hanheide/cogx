%%
% Originally a part of: curiousDanijel (developed within EU project CogX)
% Author: Matej Kristan, 2009 (matej.kristan@fri.uni-lj.si; http://vicos.fri.uni-lj.si/matejk/)
% Last revised: 2009
%%
function showSurfaceFromPoints( x )

x(:,3) = -x(:,3) ;

TRI = delaunay(x(:,1),x(:,2)); 
trisurf(TRI,x(:,1),x(:,2),x(:,3), x(:,3)) ;
colormap bone ;
hold on ;
plot3(x(:,1),x(:,2),x(:,3),'r.') ;
axis equal ; axis tight ; grid off ;

view([37, 42])