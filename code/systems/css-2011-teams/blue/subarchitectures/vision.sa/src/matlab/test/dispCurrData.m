%function dispCurrData

global Data;
global Params;

global FigDebug
if isempty(FigDebug)
   FigDebug=figure;
else
   figure(FigDebug);
end

[x,b,pt3d]=readImage(Data.currImg);

disp(['currImg: ' num2str(Data.currImg)]);
%dfigure;

submax = 4 ;
subplot(1,submax,1);imshow(x);
subplot(1,submax,2);imshow(b);

f=extAPfeatures(x,b,Params.FV,pt3d);
f'
rgb=hsv2rgb(f(1:3)');

y(:,:,1)=ones(100,100)*rgb(1);
y(:,:,2)=ones(100,100)*rgb(2);
y(:,:,3)=ones(100,100)*rgb(3);
subplot(1,submax,3);imshow(y);
subplot(1,submax,4); hold off; showSurfaceFromPoints( pt3d ) ; 

