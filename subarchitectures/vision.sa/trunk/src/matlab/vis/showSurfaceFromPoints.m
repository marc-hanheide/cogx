%%
% Originally a part of: curiousDanijel (developed within EU project CogX)
% Author: Matej Kristan, 2009 (matej.kristan@fri.uni-lj.si; http://vicos.fri.uni-lj.si/matejk/)
% Last revised: 2009
%%
function showSurfaceFromPoints( x, rgb3d )

if nargin < 2
    rgb3d = [] ;
end

x(:,3) = -x(:,3) ;

TRI = delaunay(x(:,1),x(:,2));
if isempty(rgb3d)
    trisurf(TRI,x(:,1),x(:,2),x(:,3)) ;
    colormap bone ;
else
    trisurf(TRI,x(:,1),x(:,2),x(:,3),[1:size(x(:,3),1)]') ;
    colormap(rgb3d/255) ;
    shading interp ;
end
hold on ;
plot3(x(:,1),x(:,2),x(:,3),'r.') ;
axis equal ; axis tight ; grid off ;

view([37, 42])