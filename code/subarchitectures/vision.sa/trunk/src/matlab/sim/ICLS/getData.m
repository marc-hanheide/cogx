loadData=1;

%%%%%%%%%%%% LOAD DATA %%%%%%%%%%%%%%%%%%%
if loadData %load new data

   if readData
      getAllData2;
      %filterData;
   end

   if loadSavedData
      %load C, F, Cnames, Fnames
      
      load(dataFile);
   end

   %all images
   Nall=size(F,2);
   imgIdx=1:Nall;
   numC=size(C,1);
   if randAll
      disp('RAND!');
      pidxs=randperm(Nall);
      F=F(:,pidxs);
      C=C(:,pidxs);
   end

   %training images
   trImgs=1:N;
   trImgIdx=imgIdx(trImgs);
   Ftr=F(:,trImgs);
   Ctr=C(:,trImgs);
   Ctrsf=lf2sf(Ctr);
   if randTrain
      pidxs=randperm(N);
      Ftr=Ftr(:,pidxs);
      Ctr=Ctr(:,pidxs);
      Ctrsf=Ctrsf(:,pidxs);
   end

   %test images
   tImgs=N+1:N+Nt;
   tImgIdx=imgIdx(tImgs);
   Ft=F(:,tImgs);
   Ct=C(:,tImgs);
   Ctsf=lf2sf(Ct);

end %end load new data
