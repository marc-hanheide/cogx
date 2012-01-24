function [mC,mCG,mFS]=MTDbatch(Ftr,Ctr)


global MTD;

mCG=[];mFS=[];

switch MTD
   case 1
      [mC,mCG,mFS]=MVBFbatch(Ftr,Ctr);
   case 2
      warning('off','optim:fmincon:SwitchingToMediumScale');
      [mC,mCG,mFS]=KDBFbatch(Ftr,Ctr);
   case 3
      mC=MKDEbatch(Ftr,Ctr);
   case 4
      mC=ABbatch(Ftr,Ctr);
   case 5
      mC=MKDBFbatch(Ftr,Ctr);
end

  