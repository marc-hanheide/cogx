function showROI(x,b,f,pts3d)

global Settings
global LRaxRoi LRtxFroi LRaxRec LRtxRec LRtxFrec LRtxFisRec LRtxFisRoi LRaxPts3d
global Dirs

cla(LRaxRec);
set(LRaxRec,'Visible','off');
set(LRtxRec,'String','');
set(LRtxFrec,'String','');
set(LRaxRoi,'Visible','off');
set(LRtxFisRec,'Visible','off');
set(LRtxFisRoi,'Visible','off');
set(LRaxPts3d,'Visible','off');

uselocalcolorifok = 0 ; 

if nargin>0
   x1=cutBG(x,b);

   %imshow(x1,'Parent',LRaxRoi);
   image(size(x1,1),size(x1,2),x1,'Parent',LRaxRoi);
   axis(LRaxRoi,'equal');
   axis(LRaxRoi,'off');        

   %set(LRtxFisRoi,'Visible','on');         
   %%set(LRtxFroi,'String',[num2str(f','%.2g  ') ' ]']);
  
   %set(LRaxPts3d,'Visible','on');
   %axes(LRaxPts3d) ;
   
   if ~isempty(pts3d)
       if size(pts3d,2) < 6
           ptcol = repmat(hsv2rgb(f(1:3)')*255, size(pts3d,1),1) ;
       else
           % ptcol = pts3d(:,4:6) ;
           ptcol = pts3d(:,[6,5,4]) ;
       end
       if uselocalcolorifok == 1
            ptcol = repmat( hsv2rgb(f(1:3)')*255, size(pts3d,1), 1 ) ;
       end
       showSurfaceFromPoints( pts3d(:,1:3), ptcol, LRaxPts3d ) ;
   end
   
   if Settings.SaveImgs
      Settings.Params.ImgNo=Settings.Params.ImgNo+1;
      imwrite(x,[Dirs.images 'img' num2str(Settings.Params.ImgNo,'%03d') ,'.png']);
      imwrite(b,[Dirs.images 'msk' num2str(Settings.Params.ImgNo,'%03d') ,'.png']);
   end
   
end

