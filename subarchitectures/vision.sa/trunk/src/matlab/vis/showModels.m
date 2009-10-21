function showModels(mC,Cnames,Fnames,sphw)

%return;

global MTD;

if nargin<4
   sphw=[3 4];
end

numC=length(mC);

switch MTD
   case 4
      for i=1:numC
         subplot(sphw(1),sphw(2),i)
         pdf.mu=mC(i).mean;
         pdf.covariances=mC(i).var;
         pdf.weights=1;
         showDecomposedPdf(pdf);
         title([Cnames(mC(i).name,:) ' - ' Fnames(mC(i).Fb,:) ' n=' num2str(mC(i).conf)]);
%          title([Cnames(mC(i).name,:)]);
%          xlabel(Fnames(mC(i).Fb,:));

         %axis tight;
      end;

   case {3, 9}
      for i=1:numC
         subplot(sphw(1),sphw(2),i)
         showDecomposedPdf(mC(i));
         %title([Cnames(mC(i).name,:) ' - ' Fnames(mC(i).Fb,:) ' n=' num2str(mC(i).conf)]);
          set(gca,'FontSize',16);
         title([Cnames(mC(i).name,:)]);
         xlabel(Fnames(mC(i).Fb,:));
         %plot(mC(i).max.pos,mC(i).max.val,'go');
         axis tight;
      end;

end