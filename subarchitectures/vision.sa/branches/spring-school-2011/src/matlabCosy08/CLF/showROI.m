function showROI(x,b,f)

global LRaxRoi LRtxFroi LRaxRec LRtxRec LRtxFrec LRtxFisRec LRtxFisRoi
global SaveImgs ImgNo Dirs

cla(LRaxRec);
set(LRaxRec,'Visible','off');
set(LRtxRec,'String','');
set(LRtxFrec,'String','');
set(LRaxRoi,'Visible','off');
set(LRtxFisRec,'Visible','off');
set(LRtxFisRoi,'Visible','off');

if nargin>0


   x1=cutBG(x,b);

   imshow(x1,'Parent',LRaxRoi);

   set(LRtxFisRoi,'Visible','on');
   set(LRtxFroi,'String',[num2str(f','%.2g  ') ' ]']);

   if SaveImgs
      ImgNo=ImgNo+1;
      imwrite(x,[Dirs.images 'img' num2str(ImgNo,'%03d') ,'.png']);
      imwrite(b,[Dirs.images 'msk' num2str(ImgNo,'%03d') ,'.png']);
   end
   
end
