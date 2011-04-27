function showROI(x,b,f,pts3d)

%%return;%%TODEBUG
global Settings Figs
global Dirs

% disp('showROI');
% Figs
% disp('end Figs');

cla(Figs.LRguiR.LRaxRec);
set(Figs.LRguiR.LRaxRec,'Visible','off');
set(Figs.LRguiR.LRtxRec,'String','');
set(Figs.LRguiR.LRtxFrec,'String','');
set(Figs.LRguiR.LRaxRoi,'Visible','off');
set(Figs.LRguiR.LRtxFisRec,'Visible','off');
set(Figs.LRguiR.LRtxFisRoi,'Visible','off');
set(Figs.LRguiR.LRaxPts3d,'Visible','off');

uselocalcolorifok = 0 ; 

if nargin>0

%     sizex=size(x);
%     if ~isequal(sizex(1:2),size(b))
%         b=imresize(b,sizex(1:2), 'nearest');
%     end
    
    x1=cutBG(x,b);

   %imshow(x1,'Parent',LRaxRoi);
   image(size(x1,1),size(x1,2),x1,'Parent',Figs.LRguiR.LRaxRoi);
   axis(Figs.LRguiR.LRaxRoi,'equal');
   axis(Figs.LRguiR.LRaxRoi,'off');        

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
%%TODEBUG       
showSurfaceFromPoints( pts3d(:,1:3), ptcol, Figs.LRguiR.LRaxPts3d ) ;
   end
   
   if Settings.SaveImgs
      Settings.Params.ImgNo=Settings.Params.ImgNo+1;
      imwrite(x,[Dirs.images 'img' num2str(Settings.Params.ImgNo,'%03d') ,'.png']);
      imwrite(b,[Dirs.images 'msk' num2str(Settings.Params.ImgNo,'%03d') ,'.png']);
   end
   
end

