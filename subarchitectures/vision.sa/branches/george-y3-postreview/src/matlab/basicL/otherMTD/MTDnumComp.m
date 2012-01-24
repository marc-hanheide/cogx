function nc=MTDnumComp(mC)

global MTD;

switch MTD
   case 1
      nc=1;
   case {2,3,5}
      kdes=[mC.kde];
      if isempty(kdes)
         nc=1;
      else
         pdfs=[kdes.pdf];
         nc=length([pdfs.w])/length(mC);
      end
   case 4
      nc=length([mC.Weights])/length(mC);
   case 6
      nc=getc(mC,'numComp');
      
end



