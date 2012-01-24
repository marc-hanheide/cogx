function showCmodels(mAV,Fnames,avAcronyms,ax)
%showAssoc(mAV,Fnames,avAcronyms)
%Show associations between features and atribute values.
%mAV: model of AVs
%Fnames: feature names
%avAcronyms: acronyms of AVs

if nargin<4
   ax=gca
end;

cla(ax);
set(ax,'PlotBoxAspectRatioMode','auto', ...
   'DataAspectRatioMode'   ,'auto', ...
   'CameraViewAngleMode'   ,'auto');
set(ax,'XTickLabel',Fnames);
set(ax,'Xlim',[.5,6.5]);
set(ax,'Ylim',[-.05,1.05]);

set(ax,'Color','none');
set(ax,'Box','on');

numF=size(Fnames,1);
numAV=size(avAcronyms,1);

numcAV=size(mAV,2);
names=[mAV.name];
Fbs=[mAV.Fb];
confs=[mAV.conf];

PAR=2;
for i=1:numcAV
   
   if mAV(i).conf>1
      line([mAV(i).Fb-.2,mAV(i).Fb+.2],[mAV(i).mean,mAV(i).mean],'Parent',ax);
      %   line([mAV(i).Fb-.4,mAV(i).Fb+.4],[mAV(i).mean-sqrt(mAV(i).var),mAV(i).mean-sqrt(mAV(i).var)]);
      %   line([mAV(i).Fb-.4,mAV(i).Fb+.4],[mAV(i).mean+sqrt(mAV(i).var),mAV(i).mean+sqrt(mAV(i).var)]);
      text(mAV(i).Fb+.2,mAV(i).mean,avAcronyms(names(i),:),'Parent',ax);
      rectangle('Position',[mAV(i).Fb-2*PAR*sqrt(mAV(i).var),mAV(i).mean-PAR*sqrt(mAV(i).var),4*PAR*sqrt(mAV(i).var+1e-10),2*PAR*sqrt(mAV(i).var+1e-10)],'Curvature',[1,1],'EdgeColor','blue','Parent',ax);
   end
end

