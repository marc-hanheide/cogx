%testIncrTD
%test incremental learing of concepts
%MVBNF incremental, normalized with var
%real shape images
%10C, 6F
%to be used with setGvars or testMult

%global variables, use setGvars to set
global loadData showRes N0ts N0ex THRs MTD
% loadData=1;
% showRes=1;
% N0=10
% THRs=[1.65 2.05 3.10] % 95% 98% 99.9%
THRst=THRs
%THRst=[1.6 1.8 2]%THRs

readConstants;

loadData=1;
showRes=1;
readData=0;
loadSavedData=0;
readRelData=0;
loadSavedRelData=0;
readShpData=1;
genRelData=0;
randAll=1;
randTrain=0;
resEER=0;

MTD=3


%%%%%%%%%%%% LOAD DATA %%%%%%%%%%%%%%%%%%%
if loadData %load new data

   if readData
      getAllData;
      filterData;
   end

   if loadSavedData
      %load data;
      load C:\danijels\Matlab\cogLearn\data\objects\dataAP500.mat
   end

   if readRelData
      getAllRelData;
   end

   if loadSavedRelData
      load relData;
   end;

   if readShpData
      load dataShapes300;
   end


   Nall=size(F,2);
   imgIdx=1:Nall;
   numC=size(C,1);
   
   
   N=250;
   Nt=50;


   %    trImgs=drand(N,1:Nall);
   %    restImgs=rest(1:Nall,trImgs);
   %    tImgs=restImgs(drand(Nt,restImgs));
   %    trImgs=trImgs(randperm(length(trImgs)));

   if genRelData
      Nall=600;
      imgIdx=1:Nall*2;
      P=genrandPairs(Nall);
      C=assignRel(P);
      F=extRelFeatures(P);
      numC=11;
      Cnames=['TL';'TR';'CT';'FT';'NT';'FF';'OL';'IM';'OR';'NR';'FA'];
      Fnames=['x ';'y ';'dx';'dy';'d '];
   end;

   %       C=C([1 2 11],:);
   %       numC=3;
   %       Cnames=['TL';'TR';'FA'];
   %       Fnames=['x ';'y ';'dx';'dy';'d '];

   if randAll
      disp('RAND!');
      pidxs=randperm(Nall);
      F=F(:,pidxs);
      C=C(:,pidxs);
   end

   N=200;

   trImgs=1:N;
   tImgs=N+1:Nall;


   N=length(trImgs);
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


   Nt=length(tImgs);
   tImgIdx=imgIdx(tImgs);
   Ft=F(:,tImgs);
   Ct=C(:,tImgs);
   Ctsf=lf2sf(Ct);

end %end load new data



%inicialize data for displaying learning in progress
testNs=1:5:N;%16;
%testNs=[1:10 2:10:N];%16;
testNs=[2:N0ex-1 N0ex:5:N];
testNs=[1:19 20:5:N];
testNs=[1 5:5:N];
%testNs=[20:10:N];

RS=zeros(1,length(testNs));
NQ=zeros(1,length(testNs));
RT=zeros(1,length(testNs));
TNQ=zeros(1,length(testNs));
NC=zeros(1,length(testNs));
Tn=zeros(1,length(testNs));
RR=zeros(1,length(testNs));
TPF=zeros(1,length(testNs));
TNF=zeros(1,length(testNs));
NG=zeros(1,length(testNs));
j=0;

if showRes
   figRS=dfigure(4,1,'RS');
   figRR=dfigure('RR');
   figNG=dfigure('NG');
   figNC=dfigure('NC');
   figMD=figure;
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

      %          figure;
      %    showDecomposedPdf(mCtd(1));
      %    hold off;
      %    drawnow;


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

         figure(figRS);
         plot(Tn(1:j),RS(1:j),'.g-');

         figure(figRR);
         plot(Tn(1:j),RR(1:j),'.g-');

         figure(figNG);
         plot(Tn(1:j),NG(1:j),'.g-');

         figure(figNC);
         plot(Tn(1:j),NLC(1:j),'.g-');hold on;
         plot(Tn(1:j),NEC(1:j),'.g-');
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

   dfigure(5,2,0);
   plotRes(Ct,rCtd,resCtd,Cnames,'TD: ');
   
   dfigure('Models');showLmodels(mCtd,Fnames,Cnames);


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