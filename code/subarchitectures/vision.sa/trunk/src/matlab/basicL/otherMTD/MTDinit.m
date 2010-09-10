function [mC,mCG,mFS]=MTDinit

global MTD;

mCG=[];mFS=[];

switch MTD
   case 1
      disp('MVBF model initialized.');
      [mC,mCG,mFS]=MVBFinit;
   case 2
      disp('KDBF model initialized.');
      [mC,mCG,mFS]=KDBFinit;
   case 5
      disp('MKDBF model initialized.');
      [mC]=MKDBFinit;
   case 6
      disp('ODKDE model initialized.');
      mC=ODKDEinit;
end
