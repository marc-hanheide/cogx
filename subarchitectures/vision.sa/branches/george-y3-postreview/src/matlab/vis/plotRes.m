function plotRes(AVgt,rAV,res,names,msg)

if nargin<5 msg=[]; end;

if (max(rAV(:,1))>1)
   rAV=rAV(:,2:end);
end;   
if (max(AVgt(:,1))>1)
   AVgt=AVgt(:,2:end);
end;


%confusion matrix
dfigure([msg 'CMc']);
showImg(res.CM1);
xtickLabels(names);ytickLabels(names);

%ground truth
dfigure([msg 'gtC']);
showImg(AVgt);
set(gca,'Clim',[0,1]);
colormap(hot)
ytickLabels(names);
set(gcf,'WindowButtonDownFcn',['[idxs,foo,foo]=impixel;disp(idxs'');IDXS=idxs;']);

%recognized attribute values
dfigure([msg 'rC']);
showImg(rAV);
set(gca,'Clim',[0,1]);
colormap(hot)
ytickLabels(names);
set(gcf,'WindowButtonDownFcn',['[idxs,foo,foo]=impixel;disp(idxs'');IDXS=idxs;']);

%recognition score
dfigure([msg 'RS']);
showImg(res.RS);
set(gca,'Clim',[-1,1]);
colormap(hot)
ytickLabels(names);
set(gcf,'WindowButtonDownFcn',['[idxs,foo,foo]=impixel;disp(idxs'');IDXS=idxs;']);

%error
dfigure([msg 'ER']);
showImg(rAV-AVgt);
set(gca,'Clim',[-1,1]);
colormap(hot)
ytickLabels(names);
set(gcf,'WindowButtonDownFcn',['[idxs,foo,foo]=impixel;disp(idxs'');IDXS=idxs;']);
