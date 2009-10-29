function VSnextImg

global axCimgH axCpts3dH;
global Data;
global Params ;


deterministic = 0 ; 

if deterministic == 0
    Data.currImg=ceil(rand*Data.numImgs);
else
    Data.currImg = Data.currImg + 1 ;
end


[x,b,pt3d]=readImage(Data.currImg);

imshow(x,'Parent',axCimgH)
set(axCimgH,'Visible','off');

if ~isempty(pt3d)
%     figure(7) ; clf ;
    axes(axCpts3dH) ; hold off ;
    f=extAPfeatures(x,b,Params.FV);
 
    showSurfaceFromPoints( pt3d, repmat( hsv2rgb(f(1:3)')*255, size(pt3d,1), 1 ) ) ;
end

ATinterface;