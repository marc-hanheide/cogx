%test2

%use cmlSim to set parameters


%%%%%%%%%%%% LOAD DATA %%%%%%%%%%%%%%%%%%%
if loadData %load new data

   if readData
      getAllData;
      filterData;
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



%inicialize data for displaying learning in progress
testNs=1:5:N;%16;

%Initialize result matrix
%1  1  3  4   5   6  7  8   9  10  11
%Tn,RS,RR,TPF,TNF,NG,NQ,TNQ,RT,NEC,NLC
RES=zeros(11,length(testNs));

j=0;

if showRes
   figRes=dfigure(6,1,'Evolution of results');
   resizeFigs(figRes,6,1);
   figRes2=dfigure(6,2,'Current results');
   resizeFigs(figRes2,6,1);
end
tnumqTD=0;


%%%%%%%%%%%%%%%% INITIALIZATION %%%%%%%%%%%%%%

[mCtd,mCGtd,mFStd]=MTDinit;


%%%%%%%%%%%%%%%% INCREMENTAL LEARNING %%%%%%%%%%%%%%

for i= 1:N

   if showRes
      dwaitbar(i/N,'Incremental learning...');
   end;
   if pauseNow
      disp('Paused!');drawnow;
      while pauseNow
         pause(.1);
      end
      disp('Resumed!');
   end;

   f=Ftr(:,i);
   cgt=Ctrsf(:,i);
   cgt=cgt((cgt~=0));

   %TD
%    c=cgt;
%    [mCtd,mCGtd,mFStd]=MTDupdate(f,c,mCtd,mCGtd,mFStd);
%    numqTD=length(c);
%    tnumqTD=tnumqTD+numqTD;
%    [nlcTD,necTD]=numConcepts(mCtd);
   
   LM=1;
   [mCtd,mCGtd,mFStd,numqTD]=LMupdate(LM,f,cgt,mCtd,mCGtd,mFStd);
   
   tnumqTD=tnumqTD+numqTD;
   [nlcTD,necTD]=numConcepts(mCtd);
   



   %%%%%% T E S T  %%%%%%

   if ismember(i,testNs) %test and show progress

      %TD
      rCqnt=MTDrec(Ft,mCtd,mFStd);
      rC=qnt2ql(rCqnt,THRst);
      rCtd=lf2lof(rC,numC);
      resCtd=evalRes(rCtd,Ct);
      rtTD=calcRT(resCtd.rs,tnumqTD);

      %pack current results
      j=j+1;
      RES(:,j)=[i;resCtd.rs;resCtd.rr1;resCtd.tpf1;resCtd.tnf1;0;numqTD;tnumqTD;rtTD;necTD;nlcTD];
      if MTD==3
         RES(6,j)=length([mCtd.weights])/length(mCtd);
      else
         RES(6,j)=1;
      end

      %plot current state
      if showRes
         
         figure(figRes);
         subplot(1,4,1);
         plot(RES(1,1:j),RES(2,1:j),'.g-');
         title('RS');

         subplot(1,4,2);
         plot(RES(1,1:j),RES(3,1:j),'.g-');
         title('RR');

         subplot(1,4,3);
         plot(RES(1,1:j),RES(6,1:j),'.g-');
         title('NG');

         subplot(1,4,4);
         plot(RES(1,1:j),RES(11,1:j),'.g-');hold on;
         plot(RES(1,1:j),RES(10,1:j),'.g-');
         title('NC');
         
         figure(figRes2);
         plotRes1(Ct,rCtd,resCtd,Cnames,'TD: ');
                  
      end

   end;


   if 1==0
      figure(figMD);
      clf;
      showmodels(mCtd,Cnames,Fnames);
      drawnow;
      %waitforbuttonpress;
   end

end



%print the final results
if showRes
   printRes(resCtd,'Ctd:  ',{'tpf1s','tnf1s'});

   disp('*** TD ***');printCG(mCGtd,Fnames,Cnames);

   %dfigure('Models');showLmodels(mCtd,Fnames,Cnames);


end


if resEER

   disp('*** Results at EER ***');

   %   [eer,thr]=findEER(mCtd,mFStd,Ft,Ct);
   [eer,thr]=findEER(mCtd,mFStd,Ft,Ct,[0 3]);
   disp(['THR at EER: ' num2str(thr)]);

   rCqnt=MTDrec(Ft,mCtd,mFStd);
   rC=qnt2ql(rCqnt,[1 1 1]*thr);
   rCtd=lf2lof(rC,numC);
   resCtd=evalRes(rCtd,Ct);

   printRes(resCtd,'Ctd:  ',{'tpf1s','tnf1s'});

   figRes3=dfigure(6,3,'Results at EER');
   resizeFigs(figRes3,6,1);
   plotRes1(Ct,rCtd,resCtd,Cnames,'TD: ');
end
