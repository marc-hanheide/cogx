function VSgenColImg

global Figs
global Data;
global Params ;

wh=11;

global Test
%rgb=impixel(rgba);rgb
[i j]=getpts(Test.rgba);
pt=round([i,j]);
rgb=squeeze(Test.RGB(pt(2),pt(1),:))

x=cat(3,repmat(rgb(1),wh,wh),repmat(rgb(2),wh,wh),repmat(rgb(3),wh,wh));
b=ones(wh,wh);
pt3d=[0 0 0; 1 0 0; 0 1 0; 1 1 0];

Test.x=x;
Test.b=b;
Test.pt3d=pt3d;


%[x,b,pt3d]=readImage(Data.currImg);
N_max = min([100, size(x,1)]) ;

imshow(x,'Parent',Figs.vsHs.axCimgH)
set(Figs.vsHs.axCimgH,'Visible','off');

if ~isempty(pt3d)
%     figure(7) ; clf ;
    axes(Figs.vsHs.axCpts3dH) ; hold off ;
    f=extAPfeatures(x*255,b,Params.FV);
    
    cll = repmat( hsl2rgb(f(1:3)')*255, size(pt3d,1), 1 ) ;
    idx = round(linspace(1,size(pt3d,1),N_max)) ;
    pt3d = pt3d(idx,:) ;
    cll = cll(idx,:) ;
    
    showSurfaceFromPoints( pt3d, cll  ) ;
end

 msg = sprintf('Img %d has been read from the disk!',Data.currImg) ; disp(msg) ;

Test.genColImg=1;
ATinterface;

rotate3d(Figs.vsHs.axCpts3dH,'on') ;
% 
  