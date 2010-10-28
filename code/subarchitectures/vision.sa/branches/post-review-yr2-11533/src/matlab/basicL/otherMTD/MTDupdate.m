function    [mCtd,mCGtd,mFStd]=MTDupdate(f,c,mCtd,mCGtd,mFStd)


global MTD;

switch MTD
   case 1
      [mCtd,mCGtd,mFStd]=MVBFupdate(f,c,mCtd,mCGtd,mFStd);
   case 2
      warning('off','optim:fmincon:SwitchingToMediumScale');
      [mCtd,mCGtd,mFStd]=KDBFupdate(f,c,mCtd,mCGtd,mFStd);
   case 5
      [mCtd]=MKDBFupdate(f,c,mCtd);
   case 6
      [mCtd]=ODKDEupdate(f,c,mCtd);
end

  