function VSnextImg

global axCimgH;
global Data;

Data.currImg=ceil(rand*Data.numImgs);
[x,b,pt3d]=readImage(Data.currImg);

imshow(x,'Parent',axCimgH)
set(axCimgH,'Visible','off');

if ~isempty(pt3d)
    figure(7) ; clf ;
    showSurfaceFromPoints( pt3d ) ;
end

ATinterface;