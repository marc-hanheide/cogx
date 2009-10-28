function showROI(x,b,f,pts3d)

global LRaxRoi LRtxFroi LRaxRec LRtxRec LRtxFrec LRtxFisRec LRtxFisRoi LRaxPts3d
global SaveImgs ImgNo Dirs

cla(LRaxRec);
set(LRaxRec,'Visible','off');
set(LRtxRec,'String','');
set(LRtxFrec,'String','');
set(LRaxRoi,'Visible','off');
set(LRtxFisRec,'Visible','off');
set(LRtxFisRoi,'Visible','off');
set(LRaxPts3d,'Visible','off');

 

if nargin>0


   x1=cutBG(x,b);

   %imshow(x1,'Parent',LRaxRoi);
   image(size(x1,1),size(x1,2),x1,'Parent',LRaxRoi);
   axis(LRaxRoi,'equal');
   axis(LRaxRoi,'off');        

   set(LRtxFisRoi,'Visible','on');         
   set(LRtxFroi,'String',[num2str(f','%.2g  ') ' ]']);
  
   set(LRaxPts3d,'Visible','on');
   axes(LRaxPts3d) ;
   if size(pts3d,2) < 6
       ptcol = [] ;
   else
       ptcol = pts3d(:,4:6) ;
   end
   showSurfaceFromPoints( pts3d(:,1:3), ptcol, LRaxPts3d ) ;
   
   if SaveImgs
      ImgNo=ImgNo+1;
      imwrite(x,[Dirs.images 'img' num2str(ImgNo,'%03d') ,'.png']);
      imwrite(b,[Dirs.images 'msk' num2str(ImgNo,'%03d') ,'.png']);
   end
   
end

% dispCurrData; %DEBUG!!!