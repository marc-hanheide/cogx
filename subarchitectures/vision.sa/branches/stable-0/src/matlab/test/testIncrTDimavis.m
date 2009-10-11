%testIncrTDimavis
%test incremental learing and unlearning of concepts
%for IMAVIS paper

%%%%%%%%%%%% SET PARAMETERS %%%%%%%%%%%%%%%%%%%
setParams=1;
if setParams
   
   %SWITCHES
   loadData=1;
   readData=0;
   loadSavedData=1;
   genRelData=0;
   randAll=1;
   randTrain=0;

   showRes=1;
   makeAVI=0;
   resEER=0;

   %PARAMETERS
   global MTD;
   MTD=3;
   THRs=[1 1 1]*.05;
   THRst=THRs;

   N=150; %number of training images
   Nt=150; %number of test images

   testNs=[1 5:5:N];
   Nerrs=[];%10:10:70;
   Nul=750;

   aviFname='test4.avi';

   readConstants;

end;



if makeAVI
   aviFname='testIMAVIS3';
   aviobj = avifile(aviFname,'fps',1);
end;


%%%%%%%%%%%% LOAD DATA %%%%%%%%%%%%%%%%%%%
if loadData %load new data

   if readData
      getAllData;
      filterData;
   end

   if loadSavedData
      load C:\danijels\Matlab\cogLearn\data\objects\dataAP500.mat
%       Nall=size(F,2);
%       numC=size(C,1);
%       imgIdx=1:Nall;
   end

   Nall=size(F,2);
   imgIdx=1:Nall;
   numC=size(C,1);

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



%%%%%%%%% INIT VARIABLES AND FIGURES %%%%%%%%%%%%

Ful=[];
Cul=[];

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
   %figMD=dfigure(5,3,'Models');
   figMD=dfigure(8,3,'Models');
   %resizeFigs(figMD,5,2);
   resizeFigs(figMD,8,1);
   figFM=dfigure(4,4,'Features - Models');
   resizeFigs(figFM,4,1);
   figFS=dfigure(7,3,'FS');
   resizeFigs(figFS,2,2);
end
resNaN=struct('rs',NaN,'rr1',NaN,'rr2',NaN);
tnumqTD=0;

if makeAVI
   figAVI=dfigure(7,3);
   resizeFigs(figAVI,7,3);
end


%%%%%%%%%%%%%%%% INITIALIZATION %%%%%%%%%%%%%%

[mCtd,mCGtd,mFStd]=MTDinit;


%%%%%%%%%%%%%%%% INREMENTAL LEARNING %%%%%%%%%%%%%%

for i= 1:N
   
   disp(i);
   
   if showRes
      dwaitbar(i/N,'Incremental learning...');
   end;

   f=Ftr(:,i);
   cgt=Ctrsf(:,i);
   cgt=cgt((cgt~=0));

   %Tutor driven learning
   if ~ismember(i,Nerrs)
      c=cgt;
   else
      c=Ctrsf(:,i-1);
      c=c((c~=0));
      cul=c(~ismember(c,cgt));
      if showRes
         disp(['i=' num2str(i) '   cgt=' num2str(cgt') '   c=' num2str(c') '   cul=' num2str(cul')]);
      end;   
      if ~isempty(cul)
         Ful=[Ful f];
         Cul=[Cul,sf2lf(cul,6)];
      end;   
   end
   
   [mCtd,mCGtd,mFStd]=MTDupdate(f,c,mCtd,mCGtd,mFStd);
   numqTD=length(c);
   tnumqTD=tnumqTD+numqTD;
   [nlcTD,necTD]=numConcepts(mCtd);
   
   if i==Nul      
      [mCtd,mCGtd,mFStd]=KDBFunlearn(Ful,Cul,mCtd,mCGtd,mFStd);
   end;   
   
   

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


   if showRes
      figure(figMD);
      clf;
      %showLmodels(mCtd,Fnames,Cnames);
      showLmodels(mCtd,Fnames,Cnames,[1 6]);
      drawnow;
      figure(figFM);
      clf;
      showFmodels(mCtd,Fnames,Cnames,1);
      drawnow;
%       figure(figFS);
%       clf;
%       showFS(mFStd,mCtd,Fnames,Cnames);
%       drawnow;
      %waitforbuttonpress;
   end
   
   if ismember(i,[ 70 75])
      disp(i);
   end

end



%% print the final results
if showRes 
   printRes(resCtd,'Ctd:  ',{'tpf1s','tnf1s'});

   disp('*** TD ***');printCG(mCGtd,Fnames,Cnames);

   dfigure(5,2,0);
   plotRes(Ct,rCtd,resCtd,Cnames,'TD: ');

end


%% EER
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

%%

if makeAVI
   aviobj=close(aviobj);
end   


return;
















%% new threshold
THRst=[1 1 1]*.99
rC=qnt2ql(MTDrec(Ft,mCtd,mFStd),THRst);
rCtd=lf2lof(rC,numC);
resCtd=evalRes(rCtd,Ct);
rtTD=calcRT(resCtd.rs,tnumqTD);
printRes(resCtd,'Ctd:  ',{'tpf1s','tnf1s'});
dfigure(5,2,0);
plotRes(Ct,rCtd,resCtd,Cnames,'TD: ');



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


























%% new threshold

THRst=[1 1 1]*.95

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

         
         
%% unlearn imavis

mC=mCtd;mCG=mCGtd;mFS=mFStd;

[mCtd,mCGtd,mFStd]=KDBFunlearn(Ftr(:,60),1,mCtd,mCGtd,mFStd);
[mCtd,mCGtd,mFStd]=KDBFunlearn(Ftr(:,[80 100]),3,mCtd,mCGtd,mFStd);
[mCtd,mCGtd,mFStd]=KDBFunlearn(Ftr(:,80),5,mCtd,mCGtd,mFStd);
[mCtd,mCGtd,mFStd]=KDBFunlearn(Ftr(:,120),[4 6]',mCtd,mCGtd,mFStd);

mCul=mCtd;mCGul=mCGtd;mFSul=mFStd;



%% plot

% RRok=RR;
% RRer=RR;
% RRul=RR;

plot(Tn(1:j),RRok(1:j),'.g-');
hold on
plot(Tn(1:j),RRer(1:j),'.r-');
plot(Tn(1:j),RRul(1:j),'.b-');
legend('correct labels','corrupted labels','unlearning','Location','SouthEast');
xlabel('image number');
ylabel('recognition rate');
