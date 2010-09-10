%%
% Originally a part of: curiousDanijel (developed within EU project CogX)
% Author: Matej Kristan, 2009 (matej.kristan@fri.uni-lj.si; http://vicos.fri.uni-lj.si/matejk/)
% Last revised: 2009
%%
function showSurfaceFromPoints( x, rgb3d, LRaxRoi )

if isempty(x)
    return ;
end

turnoffDelaunay =0;

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

rgb3d = rgb3d / 255 ;
if size( rgb3d , 2) == 3
    for i = 1 : 3 
        rgb3d(:,i) = max([0*rgb3d(:,i),rgb3d(:,i)]');
        rgb3d(:,i) = min([1+0*rgb3d(:,i),rgb3d(:,i)]');
    end
end

if turnoffDelaunay == 0
    if isempty(rgb3d)
        if ~isempty( LRaxRoi )
            set(LRaxRoi, 'NextPlot', 'replace') ;
            trisurf(TRI,x(:,1),x(:,2),x(:,3),'Parent',LRaxRoi,'EdgeAlpha', 0.3) ;  
            set(LRaxRoi, 'NextPlot', 'add') ;
        else
            hold off ;
            trisurf(TRI,x(:,1),x(:,2),x(:,3),'EdgeAlpha', 0.3) ;
            hold on ;
        end        
%         colormap bone ;
    else
        if isempty( LRaxRoi )
            hold off ;
            trisurf(TRI,x(:,1),x(:,2),x(:,3),[1:size(x(:,3),1)]','EdgeAlpha', 0.3) ;
            hold on ;
            colormap(rgb3d) ;

        else            
            set(LRaxRoi, 'NextPlot', 'replace') ;
            trisurf(TRI,x(:,1),x(:,2),x(:,3),[1:size(x(:,3),1)]','Parent',LRaxRoi,'EdgeAlpha', 0.3) ;
            set(LRaxRoi, 'NextPlot', 'add') ;
%             colormap(LRaxRoi,'bone') ;            
            colormap(LRaxRoi,rgb3d) ;


        end
        
%         colormap(rgb3d) ;
        %     shading interp ;
    end
    
else
    set(LRaxRoi, 'NextPlot', 'replace') ;
    hold off ;
end

if ~isempty( LRaxRoi )
    plot3(x(:,1),x(:,2),x(:,3),'r.','Parent',LRaxRoi) ;
 %             colormap(LRaxRoi,'bone') ;
            axis(LRaxRoi,'tight') ;
            axis(LRaxRoi,'equal') ;
            grid(LRaxRoi,'off') ;
            box(LRaxRoi,'on') ;   
            view(LRaxRoi,[37, 42]) ;
            set(LRaxRoi,'XTick',[]) ;
            set(LRaxRoi,'YTick',[]) ;
            set(LRaxRoi,'ZTick',[]) ;
else
    plot3(x(:,1),x(:,2),x(:,3),'r.') ;
    axis equal ; axis tight ; grid off ;
end

view([37, 42]) ; 
box on ; 
% view([-50, 20]) ; 
set(gca,'XTick',[]) 
set(gca,'YTick',[]) 
set(gca,'ZTick',[]) 

