function showLmodels(mC,Fnames,Cnames,hp)
%showLmodels(mC,Fnames,Cnames,hp)
%Show learned models

if nargin<4
   hp=gcf;
end;


sphw=[2 3];

numC=length(mC);

%clf(hp);

for i=1:numC
   if ~isempty(mC(1).name)
      ha=subplot(sphw(1),sphw(2),i,'Parent',hp);
      cla(ha);
      showDecPdf(mC(i),ha);
      %set(ha,'FontSize',16);
      title(ha,['{\bf' Cnames(mC(i).name,:) '}\leftrightarrow' Fnames(mC(i).Fb,:) ' (' num2str(mC(i).conf) ')']);
      set(ha,'ytick',[]);
      
      alim=axis(ha);
      xticks=[alim(1),(alim(1)+alim(2))/2,alim(2)];
      set(ha,'XTick',xticks);
      set(ha,'XTickLabel',{num2str(xticks(1),'%3.3f'),num2str(xticks(2),'%3.3f'),num2str(xticks(3),'%3.3f')})    
      
   end
end;

