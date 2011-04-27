function showAssoc(mAV,Fnames,avAcronyms,ax)
%showAssoc(mAV,Fnames,avAcronyms)
%Show associations between features and atribute values.
%mAV: model of AVs
%Fnames: feature names
%avAcronyms: acronyms of AVs

if nargin<4
   ax=gca;
end;

NPX=100;
NPY=4;
numF=size(Fnames,1);
numAV=size(avAcronyms,1);

cla(ax);
set(ax,'Xlim',[0 NPX]);
set(ax,'Ylim',[0 NPY]);
set(ax,'PlotBoxAspectRatioMode','auto', ...
   'DataAspectRatioMode'   ,'auto', ...
   'CameraViewAngleMode'   ,'auto');
set(ax,'Visible','off');

pxF=zeros(1,numF);
pxAV=zeros(1,numAV);
for i=1:numF
   pxF(i)=(i-1)*NPX/numF;
   text(pxF(i),4,Fnames(i,:), 'Parent',ax);
end;
for i=1:numAV
   pxAV(i)=(i-1)*NPX/numAV;
   text(pxAV(i),1,avAcronyms(i,:), 'Parent',ax);
end;

if ~isempty(mAV)

   numcAV=size(mAV,2);
   names=[mAV.name];
   Fbs=[mAV.Fb];
   confs=[mAV.conf];

   for i=1:numcAV
      if mAV(i).conf>1
         line([pxAV(names(i))+2,pxF(Fbs(i))+2],[1.2,3.8], 'Parent',ax);
      end
   end

   for i=1:numcAV
      if mAV(i).conf>1
         text(pxAV(names(i))-1,0,num2str(confs(i),'%02.2f'), 'Color','blue','FontSize',8,'Parent',ax);
      end;
   end;


end

