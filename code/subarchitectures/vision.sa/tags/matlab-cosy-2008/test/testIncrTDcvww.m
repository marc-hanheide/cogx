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
loadSavedData=1;
readRelData=0;
loadSavedRelData=0;
genRelData=0;
randAll=0;
randTrain=0;
resEER=0;
makeAVI=0;

%MTD=3

%aviFname='test5.avi';

if makeAVI
   aviFname='test9b';
   aviobj = avifile(aviFname,'fps',2);
end;

%%%%%%%%%%%% LOAD DATA %%%%%%%%%%%%%%%%%%%
if loadData %load new data

   if readData
      getAllData;
      filterData;
   end

   if loadSavedData
      load C:\danijels\Matlab\cogLearn\data\objects\dataAP500.mat
      Nall=size(F,2);
      numC=size(C,1);
      imgIdx=1:Nall;
      MTD=3;
      THRs=[1 1 1]*.95;
      THRst=THRs;
   end

   if readRelData
      getAllRelData;
   end

   if loadSavedRelData
      load relData;
   end;


   Nall=size(F,2);
   imgIdx=1:Nall;
   numC=size(C,1);


   N=120;%200;
   Nt=300;

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

figAVI=dfigure(7,3);
resizeFigs(figAVI,7,3);

%%%%%%%%%%%%%%%% INITIALIZATION %%%%%%%%%%%%%%

[mCtd,mCGtd,mFStd]=MTDinit;


%%%%%%%%%%%%%%%% INREMENTAL LEARNING %%%%%%%%%%%%%%

for i= 1:85%120%N

   if showRes
      dwaitbar(i/N,'Incremental learning...');
   end;

   f=Ftr(:,i);
   cgt=Ctrsf(:,i);
   cgt=cgt((cgt~=0));

   %TD
   if i<N0ex
      c=cgt;
   else
      %       rC=qnt2ql(MTDrec(f,mCtd,mFStd),THRs);
      %       c=lf2sfa(rC,ANSyes)';
      if mod(i,20)==0
         c=Ctrsf(:,i-1);
         c=c((c~=0));
         disp(['i=' num2str(i) '   cgt=' num2str(cgt') '   c=' num2str(c')]);
      else
         c=cgt;
      end
   end
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

      if makeAVI
         figure(figAVI);
         subplot(3,6,[1:3 7:9]);
         plot(Tn(1:j),RR(1:j),'.b-');
         setAxis([],[],0,100);
         xlabel('no of added samples');
         ylabel('accuracy');
         title('Accuracy')
         subplot(3,6,[4:6 10:12]);
         plot(Tn(1:j),NG(1:j),'.b-');
         setAxis([],[],0,20);
         xlabel('no of added samples');
         ylabel('average number of components');
         title('Average number of components');

         for jj=1:length(mCtd)
            subplot(3,6,12+jj);
            cla;
            showDecomposedPdf(mCtd(jj));
            title([Cnames(mCtd(jj).name,:)]);
            xlabel(Fnames(mCtd(jj).Fb,:));
            set(gca,'ytick',[]);
            axis tight;
            alim=axis;
            xticks=[alim(1),(alim(1)+alim(2))/2,alim(2)];
            set(gca,'XTick',xticks);
            set(gca,'XTickLabel',{num2str(xticks(1),'%3.3f'),num2str(xticks(2),'%3.3f'),num2str(xticks(3),'%3.3f')})
         end;

         drawnow;
         aviobj=addframe(aviobj,figAVI);
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
if showRes & 0
   printRes(resCtd,'Ctd:  ',{'tpf1s','tnf1s'});

   disp('*** TD ***');printCG(mCGtd,Fnames,Cnames);

   dfigure(5,2,0);
   plotRes(Ct,rCtd,resCtd,Cnames,'TD: ');

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



if makeAVI
   aviobj=close(aviobj);
end   


return;


%% copy models

mC=mCtd;
mFS=mFStd;
mCG=mCGtd;

THRs=[1 1 1]*.95;

rCqnt=KDBFrec(Ftr,mC,mFS);
rCqntt=KDBFrec(Ft,mC,mFS);
rCtr=qnt2ql(rCqnt,THRs);
rCt=qnt2ql(rCqntt,THRs);
%resCtr=evalRes(rCtr,Ctr);
resCtr=evalRes(lf2lof(rCtr,numC),Ctr);
resCt=evalRes(lf2lof(rCt,numC),Ct);

printRes(resCtr,'Ctr: ', {'tpf1s','tnf1s'});
printRes(resCt,'Ct : ', {'tpf1s','tnf1s'});
figure;
showmodels(mC,Cnames,Fnames);
showErr(mC,resCtr,Ftr,Ctr);
showErr(mC,resCt,Ft,Ct);



%% update with FN
[mC,mCG,mFS]=KDBFupdate(Ftr,resCtr.FN1,mC,mCG,mFS);

rCqnt=KDBFrec(Ftr,mC,mFS);
rCqntt=KDBFrec(Ft,mC,mFS);
rCtr=qnt2ql(rCqnt,THRs);
rCt=qnt2ql(rCqntt,THRs);
resCtr=evalRes(rCtr,Ctr);
resCt=evalRes(rCt,Ct);

printRes(resCtr,'Ctr: ', {'tpf1s','tnf1s'});
printRes(resCt,'Ct : ', {'tpf1s','tnf1s'});
figure;
showmodels(mC,Cnames,Fnames);
showErr(mC,resCtr,Ftr,Ctr);


%% unlearn FP

[mC,mCG,mFS]=KDBFunlearn(Ftr,resCtr.FP1,mC,mCG,mFS);

rCqnt=KDBFrec(Ftr,mC,mFS);
rCqntt=KDBFrec(Ft,mC,mFS);
rCtr=qnt2ql(rCqnt,THRs);
rCt=qnt2ql(rCqntt,THRs);
resCtr=evalRes(rCtr,Ctr);
resCt=evalRes(rCt,Ct);

printRes(resCtr,'Ctr: ', {'tpf1s','tnf1s'});
printRes(resCt,'Ct : ', {'tpf1s','tnf1s'});
figure;
showmodels(mC,Cnames,Fnames);
showErr(mC,resCtr,Ftr,Ctr);




return
























%% new threshold

THRst=[1 1 1]*.98

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

         for jj=1:length(mCtd)
            subplot(3,6,12+jj);
            cla;
            showDecomposedPdf(mC(jj));
            title([Cnames(mC(jj).name,:)]);
            xlabel(Fnames(mC(jj).Fb,:));
            set(gca,'ytick',[]);
            axis tight;
            alim=axis;
            xticks=[alim(1),(alim(1)+alim(2))/2,alim(2)];
            set(gca,'XTick',xticks);
            set(gca,'XTickLabel',{num2str(xticks(1),'%3.3f'),num2str(xticks(2),'%3.3f'),num2str(xticks(3),'%3.3f')})
         drawnow;
         end;

