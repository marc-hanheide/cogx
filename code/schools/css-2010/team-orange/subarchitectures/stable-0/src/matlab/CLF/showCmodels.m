function showCmodels(mC,Fnames,Cnames,ax)
%showAssoc(mAV,Fnames,avAcronyms)
%Show associations between features and atribute values.
%mAV: model of AVs
%Fnames: feature names
%avAcronyms: acronyms of AVs

if nargin<4
   ax=gca
end;


sphw=[2 3];

numC=length(mC);

figure(ax);
clf(ax);

for i=1:numC
   if ~isempty(mC(1).name)
      subplot(sphw(1),sphw(2),i)
      showDecomposedPdf(mC(i));
      %set(gca,'FontSize',16);
      title([Cnames(mC(i).name,:)]);
      xlabel(Fnames(mC(i).Fb,:));
      set(gca,'ytick',[]);
      axis tight;
      
      alim=axis;
      xticks=[alim(1),(alim(1)+alim(2))/2,alim(2)];
      set(gca,'XTick',xticks);
      set(gca,'XTickLabel',{num2str(xticks(1),'%3.3f'),num2str(xticks(2),'%3.3f'),num2str(xticks(3),'%3.3f')})    
      
   end
end;

