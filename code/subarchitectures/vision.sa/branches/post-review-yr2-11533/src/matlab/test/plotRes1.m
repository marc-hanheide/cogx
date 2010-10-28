function plotRes(AVgt,rAV,res,names,msg)

if nargin<5 msg=[]; end;

if (max(rAV(:,1))>1)
   rAV=rAV(:,2:end);
end;   
if (max(AVgt(:,1))>1)
   AVgt=AVgt(:,2:end);
end;



%confusion matrix
subplot(1,5,1);
showImg(res.CM1);
xTickLabels(names);ytickLabels(names);
title([msg 'CMc']);

%ground truth
subplot(1,5,2);
showImg(AVgt);
set(gca,'Clim',[0,1]);
colormap(hot)
yTickLabels(names);
set(gcf,'WindowButtonDownFcn',['[idxs,foo,foo]=impixel;disp(idxs'');IDXS=idxs;']);
title([msg 'gtC']);

%recognized attribute values
subplot(1,5,3);
showImg(rAV);
set(gca,'Clim',[0,1]);
colormap(hot)
yTickLabels(names);
set(gcf,'WindowButtonDownFcn',['[idxs,foo,foo]=impixel;disp(idxs'');IDXS=idxs;']);
title([msg 'rC']);

%recognition score
subplot(1,5,4);
showImg(res.RS);
set(gca,'Clim',[-1,1]);
colormap(hot)
yTickLabels(names);
set(gcf,'WindowButtonDownFcn',['[idxs,foo,foo]=impixel;disp(idxs'');IDXS=idxs;']);
title([msg 'RS']);

%error
subplot(1,5,5);
showImg(rAV-AVgt);
set(gca,'Clim',[-1,1]);
colormap(hot)
yTickLabels(names);
set(gcf,'WindowButtonDownFcn',['[idxs,foo,foo]=impixel;disp(idxs'');IDXS=idxs;']);
title([msg 'ER']);
