%test1

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

RS=zeros(1,length(testNs));
NQ=zeros(1,length(testNs));
RT=zeros(1,length(testNs));
TNQ=zeros(1,length(testNs));
NEC=zeros(1,length(testNs));
NLC=zeros(1,length(testNs));
Tn=zeros(1,length(testNs));
RR=zeros(1,length(testNs));
TPF=zeros(1,length(testNs));
TNF=zeros(1,length(testNs));
NG=zeros(1,length(testNs));
j=0;

if showRes
   %figMD=figure;
   figRes=dfigure(6,1);
   resizeFigs(figRes,6,1);
   figRes2=dfigure(6,2);
   resizeFigs(figRes2,6,1);

end
resNaN=struct('rs',NaN,'rr1',NaN,'rr2',NaN);
tnumqTD=0;


%%%%%%%%%%%%%%%% INITIALIZATION %%%%%%%%%%%%%%

[mCtd,mCGtd,mFStd]=MTDinit;


%%%%%%%%%%%%%%%% INREMENTAL LEARNING %%%%%%%%%%%%%%

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
   c=cgt;
   [mCtd,mCGtd,mFStd]=MTDupdate(f,c,mCtd,mCGtd,mFStd);
   numqTD=length(c);
   tnumqTD=tnumqTD+numqTD;
   [nlcTD,necTD]=numConcepts(mCtd);



   %%%%%% T E S T  %%%%%%

   if ismember(i,testNs) %test and show progress

      %TD
      rC=qnt2ql(MTDrec(Ft,mCtd,mFStd),THRst);
      rCtd=lf2lof(rC,numC);
      resCtd=evalRes(rCtd,Ct);
      rtTD=calcRT(resCtd.rs,tnumqTD);

      %pack current results
      j=j+1;
      RS(j)=resCtd.rs;
      NQ(j)=numqTD;
      TNQ(j)=tnumqTD;
      RT(j)=rtTD;
      NEC(j)=necTD;
      NLC(j)=nlcTD;
      Tn(j)=i;
      RR(j)=resCtd.rr1;
      TPF(j)=resCtd.tpf1;
      TNF(j)=resCtd.tnf1;
      if MTD==3
         NG(j)=length([mCtd.weights])/length(mCtd);
      else
         NG(j)=1;
      end



      %plot current state
      if showRes
         
         figure(figRes);
         subplot(1,4,1);
         plot(Tn(1:j),RS(1:j),'.g-');
         title('RS');

         subplot(1,4,2);
         plot(Tn(1:j),RR(1:j),'.g-');
         title('RR');

         subplot(1,4,3);
         plot(Tn(1:j),NG(1:j),'.g-');
         title('NG');

         subplot(1,4,4);
         plot(Tn(1:j),NLC(1:j),'.g-');hold on;
         plot(Tn(1:j),NEC(1:j),'.g-');
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
   thr

   rC=qnt2ql(MTDrec(Ft,mCtd,mFStd),[1 1 1]*thr);
   rCtd=lf2lof(rC,numC);
   resCtd=evalRes(rCtd,Ct);

   printRes(resCtd,'Ctd:  ',{'tpf1s','tnf1s'});

   dfigure(5,3,0);
   plotRes(Ct,rCtd,resCtd,Cnames,'TD: ');

   figure;
   showmodels(mCtd,Cnames,Fnames,[6 4]);
   showErr(mCtd,resCtd,Ft,Ct,[6 4]);
end

return






return



%% new threshold

THRst=[1 1 1]*.99

rC=qnt2ql(MTDrec(Ft,mCtd,mFStd),THRst);
rCtd=lf2lof(rC,numC);
resCtd=evalRes(rCtd,Ct);
rtTD=calcRT(resCtd.rs,tnumqTD);

printRes(resCtd,'Ctd:  ',{'tpf1s','tnf1s'});

dfigure(5,2,0);
plotRes(Ct,rCtd,resCtd,Cnames,'TD: ');


%% show models
figure;
showmodels(mCtd,Cnames,Fnames,[6 4]);
showErr(mCtd,resCtd,Ft,Ct,[6 4]);



%% add legends to figures
figure(figRS);
legend('TD','TSc','TSl','EXc','EXl');
xlabel('no of added images');
ylabel('recognition score');

figure(figNQ);
legend('TD','TSc','TSl','EXc','EXl');
xlabel('no of added images');
ylabel('number of questions');

figure(figRT);
legend('TD','TSc','TSl','EXc','EXl');
xlabel('no of added images');
ylabel('rationality score');

figure(figNC);
legend('TD','TSc','TSl','EXc','EXl');
xlabel('no of added images');
ylabel('number of learned C');
%%