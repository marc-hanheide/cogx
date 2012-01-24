function [mC,mCG,mFS]=MTDdispModel(mC,mCG,Fnames,Cnames)

global MTD;

switch MTD
   case {1,2}
      printCG(mCG,Fnames,Cnames);
   case 3   
   case 4
   case 5
      detectCG(mC,Fnames, Cnames);
%       numC=length(mC);
%       for i=1:numC
%          fprintf('%d ',mC(i).Fb)
%          fprintf('   ');
%       end
%       fprintf('\n');
   case 6
      fprintf('Selected features: ');
      disp(getc(mC,0,'Fb'));
end;
end

  