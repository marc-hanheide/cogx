function    [mCtd,mCGtd,mFStd]=MTDunlearn(f,c,mCtd,mCGtd,mFStd)


global MTD;

switch MTD
   case 1
      %[mCtd,mCGtd,mFStd]=MVBFupdate(f,c,mCtd,mCGtd,mFStd);
   case 2
      warning('off','optim:fmincon:SwitchingToMediumScale');
      [mCtd,mCGtd,mFStd]=KDBFunlearn(f,c,mCtd,mCGtd,mFStd);
   case 5
      [mCtd]=MKDBFunlearn(f,c,mCtd);
end

  