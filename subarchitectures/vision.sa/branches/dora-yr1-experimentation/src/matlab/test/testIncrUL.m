%testIncremental
%test incremental learing of concepts
%MVBNF incremental, normalized with var
%real shape images
%10C, 6F
%to be used with setGvars or testMult

%global variables, use setGvars to set
global loadData showRes N0ts N0ex THRs MTD

%MVBF
THRs=[.5 2 20];
THRst=[1.6 2 2.5];%THRs
%THRst=THRs

%KDBF
%THRs=[.92 .95 .999]
THRs=[.5 .95 .999]
THRst=[.95 .975 .99];%THRs
%THRst=THRs
THRul=THRst;

N0ts=20;
N0ex=N0ts;

readConstants;

mtdTSc=1;
mtdTSl=1;
mtdEXc=1;
mtdEXl=1;


loadData=0;
showRes=1;
readData=0;
loadSavedData=1;
genRels=0;
randTrain=1;
%MTD=3


%%%%%%%%%%%% LOAD DATA %%%%%%%%%%%%%%%%%%%
if loadData %load new data

   if readData
      getAllData;
      filterData;
   end

   if loadSavedData
      %      load data;
      load C:\danijels\Matlab\cogLearn\data\objects\dataAP500.mat
      Nall=size(F,2);
      numC=size(C,1);
      imgIdx=1:Nall;
   end


   Nall=size(F,2);

   N=200;
   Nt=50;

   %    trImgs=drand(N,1:Nall);
   %    restImgs=rest(1:Nall,trImgs);
   %    tImgs=restImgs(drand(Nt,restImgs));
   %    trImgs=trImgs(randperm(length(trImgs)));

   if genRels
      P=genrandPairs(Nall);
      C=assignRel(P);
      F=extRelFeatures(P);
      numC=11;
      Cnames=['TL';'TR';'CT';'FT';'NT';'FF';'OL';'IM';'OR';'NR';'FA'];
      Fnames=['x ';'y ';'dx';'dy';'d '];

      C=C([1 2 11],:);
      numC=3;
      Cnames=['TL';'TR';'FA'];
      Fnames=['x ';'y ';'dx';'dy';'d '];

   end
   trImgs=1:N;
   tImgs=N+1:N+Nt;

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
testNs=[1:10:N];
testNs=[1:5 5:5:20 30:10:N];

testNs=[1:4 4:5:19 29:10:N];
unlearnNs=[];%20:20:N];

testNs=unique([testNs unlearnNs]);

RS=zeros(5,length(testNs));
NQ=zeros(5,length(testNs));
RT=zeros(5,length(testNs));
TNQ=zeros(5,length(testNs));
NEC=zeros(5,length(testNs));
NLC=zeros(5,length(testNs));
Tn=zeros(1,length(testNs));
j=0;

if showRes
   figRS=dfigure(4,1,'Ct rs');
   figNQ=dfigure('NQ');
   figRT=dfigure('RT');
   figNC=dfigure('NC');
   figMD=figure;
end
resNaN=struct('rs',NaN,'rr1',NaN,'rr2',NaN);
tnumqTD=0;tnumqTSc=0;tnumqTSl=0;tnumqEXc=0;tnumqEXl=0;



%%%%%%%%%%%%%%%% INITIALIZATION %%%%%%%%%%%%%%

[mC,mCG,mFS]=MTDinit;
mCtd=mC;mCtsc=mC;mCtsl=mC;mCexc=mC;mCexl=mC;
mCGtd=mCG;mCGtsc=mCG;mCGtsl=mCG;mCGexc=mCG;mCGexl=mCG;
mFStd=mFS;mFStsc=mFS;mFStsl=mFS;mFSexc=mFS;mFSexl=mFS;


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

   %TSc
   if mtdTSc
      rC=qnt2ql(MTDrec(f,mCtsc,mFStsc),THRs);
      c=lf2sfa(rC,ANSyes)';
      [mCtsc,mCGtsc,mFStsc]=MTDupdate(f,c,mCtsc,mCGtsc,mFStsc);
      numqTSc=0;
      tnumqTSc=tnumqTSc+numqTSc;
      [nlcTSc,necTSc]=numConcepts(mCtsc);
   end;

   %TSl
   if mtdTSl
      rC=qnt2ql(MTDrec(f,mCtsl,mFStsl),THRs);
      c=lf2sfa(rC,[ANSyes,ANSpy])';
      [mCtsl,mCGtsl,mFStsl]=MTDupdate(f,c,mCtsl,mCGtsl,mFStsl);
      numqTSl=0;
      tnumqTSl=tnumqTSl+numqTSl;
      [nlcTSl,necTSl]=numConcepts(mCtsl);
   end

   %EXc
   if mtdEXc
      rC=qnt2ql(MTDrec(f,mCexc,mFSexc),THRs);
      c=lf2sfa(rC,ANSyes)';
      [mCexc,mCGexc,mFSexc]=MTDupdate(f,c,mCexc,mCGexc,mFSexc);
      numqEXc=0;
      tnumqEXc=tnumqEXc+numqEXc;
      [nlcEXc,necEXc]=numConcepts(mCexc);
   end

   %EXl
   if mtdEXl
      rC=qnt2ql(MTDrec(f,mCexl,mFSexl),THRs);
      c=lf2sfa(rC,[ANSyes,ANSpy])';
      [mCexl,mCGexl,mFSexl]=MTDupdate(f,c,mCexl,mCGexl,mFSexl);
      numqEXl=0;
      tnumqEXl=tnumqEXl+numqEXl;
      [nlcEXl,necEXl]=numConcepts(mCexl);
   end

   %copy TD models after initial N0 images
   if i==N0ts
      disp('Copy!');
      mCtsc=mCtd;mCtsl=mCtd;
      mCGtsc=mCGtd;mCGtsl=mCGtd;
      mFStsc=mFStd;mFStsl=mFStd;
      %make mCts confident
      for k=1:length(mCtsc) mCtsc(k).conf=11; end;
      for k=1:length(mCtsl) mCtsl(k).conf=11; end;
      mFStsc.Fns=ones(length(mCtsc),1)*11;
      mFStsl.Fns=ones(length(mCtsl),1)*11;
      tnumqTSc=tnumqTD;tnumqTSl=tnumqTD;
   end

   if i==N0ex
      %copy TD models into EX models
      mCexc=mCtd;mCexl=mCtd;
      mCGexc=mCGtd;mCGexl=mCGtd;
      mFSexc=mFStd;mFSexl=mFStd;
      %make mCex confident
      for k=1:length(mCexc) mCexc(k).conf=11; end;
      for k=1:length(mCexl) mCexl(k).conf=11; end;
      mFSexc.Fns=ones(length(mCexc),1)*11;
      mFSexl.Fns=ones(length(mCexl),1)*11;
      tnumqEXc=tnumqTD;tnumqEXl=tnumqTD;
   end

   
   if ismember(i,unlearnNs)
      
      disp(['Unlearn! ' num2str(i)]);
    
      
      F1i=F(:,1:i);
      C1i=C(:,1:i);
      
      rC=qnt2ql(MTDrec(F1i,mCtsc,mFStsc),THRul);
      rCtsc=lf2lof(rC,numC);
      resCtsc=evalRes(rCtsc,C1i);
      %[mCtsc,mCGtsc,mFStsc]=MVBFupdate(F1i,resCtsc.FN1,mCtsc,mCGtsc,mFStsc);
      
      %[mCtsc,mCGtsc,mFStsc]=KDBFupdate(F1i,resCtsc.FN1,mCtsc,mCGtsc,mFStsc);
      [mCtsc,mCGtsc,mFStsc]=KDBFunlearn(F1i,resCtsc.FP1,mCtsc,mCGtsc,mFStsc);

      rC=qnt2ql(MTDrec(F1i,mCtsl,mFStsl),THRul);
      rCtsl=lf2lof(rC,numC);
      resCtsl=evalRes(rCtsl,C1i);
      %[mCtsl,mCGtsl,mFStsl]=MVBFupdate(F1i,resCtsl.FN1,mCtsl,mCGtsl,mFStsl);
      
      %[mCtsl,mCGtsl,mFStsl]=KDBFupdate(F1i,resCtsl.FN1,mCtsl,mCGtsl,mFStsl);
      %[mCtsl,mCGtsl,mFStsl]=KDBFunlearn(F1i,resCtsl.FP1,mCtsl,mCGtsl,mFStsl);

   end

   %%%%%% T E S T  %%%%%%
   if ismember(i,testNs) %test and show progress

      %TD
      rC=qnt2ql(MTDrec(Ft,mCtd,mFStd),THRst);
      rCtd=lf2lof(rC,numC);
      resCtd=evalRes(rCtd,Ct);
      rtTD=calcRT(resCtd.rs,tnumqTD);


      if mtdTSc
         %TSc
         rC=qnt2ql(MTDrec(Ft,mCtsc,mFStsc),THRst);
         rCtsc=lf2lof(rC,numC);
         resCtsc=evalRes(rCtsc,Ct);
         rtTSc=calcRT(resCtsc.rs,tnumqTSc);
      end

      %TSl
      if mtdTSl
         rC=qnt2ql(MTDrec(Ft,mCtsl,mFStsl),THRst);
         rCtsl=lf2lof(rC,numC);
         resCtsl=evalRes(rCtsl,Ct);
         rtTSl=calcRT(resCtsl.rs,tnumqTSl);
      end

      %EXc
      if mtdEXc
         rC=qnt2ql(MTDrec(Ft,mCexc,mFSexc),THRst);
         rCexc=lf2lof(rC,numC);
         resCexc=evalRes(rCexc,Ct);
         rtEXc=calcRT(resCexc.rs,tnumqEXc);
      end

      %EXl
      if mtdEXl
         rC=qnt2ql(MTDrec(Ft,mCexl,mFSexl),THRst);
         rCexl=lf2lof(rC,numC);
         resCexl=evalRes(rCexl,Ct);
         rtEXl=calcRT(resCexl.rs,tnumqEXl);
      end


      %do not consider the first N0 non TD results
      if i<N0ts
         resCtsc=resNaN;resCtsl=resNaN;
         numqTSc=NaN;numqTSl=NaN;
         rtTSc=NaN;rtTSl=NaN;
      end

      if i<N0ex
         resCexc=resNaN;resCexl=resNaN;
         numqEXc=NaN;numqEXl=NaN;
         rtEXc=NaN;rtEXl=NaN;
         necEXc=NaN;nlcEXc=NaN;necEXl=NaN;nlcEXl=NaN;
      end

      %pack current results
      j=j+1;
      RS(:,j)=[resCtd.rs;resCtsc.rs;resCtsl.rs;resCexc.rs;resCexl.rs];
      NQ(:,j)=[numqTD;numqTSc;numqTSl;numqEXc;numqEXl];
      TNQ(:,j)=[tnumqTD;tnumqTSc;tnumqTSl;tnumqEXc;tnumqEXl];
      RT(:,j)=[rtTD;rtTSc;rtTSl;rtEXc;rtEXl];
      NEC(:,j)=[necTD;necTSc;necTSl;necEXc;necEXl];
      NLC(:,j)=[nlcTD;nlcTSc;nlcTSl;nlcEXc;nlcEXl];
      Tn(j)=i;

      %plot current state
      if showRes

         unNsi=unlearnNs(unlearnNs<=i);
         unIdxs=find(ismember(Tn,unNsi));
         figure(figRS);
         plot(Tn(1:j),RS(1,1:j),'.g-');hold on;%xticklabels(Tn(1:j));
         plot(Tn(1:j),RS(2,1:j),'+b--');plot(Tn(1:j),RS(3,1:j),'+b:');plot(Tn(1:j),RS(4,1:j),'xr--');plot(Tn(1:j),RS(5,1:j),'xr:');
         plot(unNsi,RS(2,unIdxs),'o');plot(unNsi,RS(3,unIdxs),'o');
         
         figure(figNQ);
         plot(Tn(1:j),NQ(1,1:j),'.g-');hold on;%xticklabels(Tn(1:j));
         plot(Tn(1:j),NQ(2,1:j),'+b--');plot(Tn(1:j),NQ(3,1:j),'+b:');plot(Tn(1:j),NQ(4,1:j),'xr--');plot(Tn(1:j),NQ(5,1:j),'xr:');

         figure(figRT);
         plot(Tn(1:j),RT(1,1:j),'.g-');hold on;%xticklabels(Tn(1:j));
         plot(Tn(1:j),RT(2,1:j),'+b--');plot(Tn(1:j),RT(3,1:j),'+b:');plot(Tn(1:j),RT(4,1:j),'xr--');plot(Tn(1:j),RT(5,1:j),'xr:');

         %          figure(figNC);
         %          plot(Tn(1:j),NC(1,1:j),'.g-');hold on;%xticklabels(Tn(1:j));
         %          plot(Tn(1:j),NC(2,1:j),'+b--');plot(Tn(1:j),NC(3,1:j),'+b:');plot(Tn(1:j),NC(4,1:j),'xr--');plot(Tn(1:j),NC(5,1:j),'xr:');

         figure(figNC);
         plot(Tn(1:j),NLC(1,1:j),'.g-');hold on;%xticklabels(Tn(1:j));
         plot(Tn(1:j),NLC(2,1:j),'+b--');plot(Tn(1:j),NLC(3,1:j),'+b:');plot(Tn(1:j),NLC(4,1:j),'xr--');plot(Tn(1:j),NLC(5,1:j),'xr:');
         plot(Tn(1:j),NEC(1,1:j),'.g-');
         plot(Tn(1:j),NEC(2,1:j),'+b--');plot(Tn(1:j),NEC(3,1:j),'+b:');plot(Tn(1:j),NEC(4,1:j),'xr--');plot(Tn(1:j),NEC(5,1:j),'xr:');

      end

      if 1==0
         figure(figMD);
         clf;
         showmodels(mCtsc,Cnames,Fnames);
         drawnow;
         %waitforbuttonpress;
      end

      
   end;

end



%print the final results
if showRes
   printRes(resCtd,'Ctd:  ');
   printRes(resCtsc,'Ctsc: ');
   %    printRes(resCtsl,'Ctsl: ');
   %    printRes(resCexc,'Cexc: ');
   %    printRes(resCexl,'Cexl: ');

   disp('*** TD ***');printCG(mCGtd,Fnames,Cnames);
   disp('*** TSc ***');printCG(mCGtsc,Fnames,Cnames);

   dfigure(5,1,0);
   plotRes(Ct,rCtd,resCtd,Cnames,'TD: ');
   plotRes(Ct,rCtsc,resCtsc,Cnames,'TSc: ');

end




return


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