function VSnextImg

global Figs
global Data;
global Params ;


if Params.deterministic == 0
    Data.currImg=ceil(rand*Data.numImgs);
else       
    if isfield(Data,'allindexes') && ~isempty(Data.allindexes)
        Data.curridximg = Data.curridximg + 1 ;
        Data.currImg = Data.allindexes(Data.curridximg) ;
    end         
end
 
[x,b,pt3d]=readImage(Data.currImg);

imshow(x,'Parent',Figs.vsHs.axCimgH)
set(Figs.vsHs.axCimgH,'Visible','off');

if ~isempty(pt3d)
%     figure(7) ; clf ;
    axes(Figs.vsHs.axCpts3dH) ; hold off ;
    f=extAPfeatures(x,b,Params.FV);
 
    showSurfaceFromPoints( pt3d, repmat( hsv2rgb(f(1:3)')*255, size(pt3d,1), 1 ) ) ;
end

 msg = sprintf('Img %d has been read from the disk!',Data.currImg) ; disp(msg) ;

ATinterface;
 
rotate3d(Figs.vsHs.axCpts3dH,'on') ;
% 
  