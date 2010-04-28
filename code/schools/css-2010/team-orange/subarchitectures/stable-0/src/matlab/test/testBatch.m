%testBatch
%test bach learing of concepts
%MVBNF batch, normalized with var
%real shape images

%global variables, use setGvars to set
global loadData THRs showRes

learn=1;
loadData=1;
readShpData=1;
readObjData=0;
genRelData=0;
%THRs=[];
MTD=9;



%MTD=3;

%Mean and variance of best normalized feature
method{1}.name='MVBNF';
method{1}.batch='[mC,mCG,mFS]=MVBNFbatch(Ftr,Ctr);';
method{1}.recTr='rCqnt=MVBNFrec(Ftr,mC,mFS);';
method{1}.recT='rCqntt=MVBNFrec(Ft,mC,mFS);';
method{1}.THRdef=[1.6 2 3];

%Null space of normalized features
method{2}.name='NSNF';
method{2}.batch='mC=NSNFbatch(Ftr,Ctr);';
method{2}.recTr='rCqnt=NSNFrec(Ftr,mC);';
method{2}.recT='rCqntt=NSNFrec(Ft,mC);';
method{2}.THRdef=[5 6 7];

%AdaBoost on images
method{3}.name='AB';
method{3}.batch='mC=ABbatch(Ftr,Ctr);';
method{3}.recTr='rCqnt=ABrec(Ftr,mC);';
method{3}.recT='rCqntt=ABrec(Ft,mC);';
method{3}.THRdef=[0 0 0];

%AdaBoost on sampled normal distribution
method{4}.name='ABSD';
method{4}.batch='mC=ABSDbatch(Ftr,Ctr);';
method{4}.recTr='rCqnt=ABrec(Ftr,mC);';
method{4}.recT='rCqntt=ABrec(Ft,mC);';
method{4}.THRdef=[0 0 0];

%Discriminative mean and variance of best normalized feature
method{5}.name='DMVBNF';
method{5}.batch='[mC,mCG,mFS]=DMVBNFbatch(Ftr,Ctr);';
method{5}.recTr='rCqnt=MVBNFrec(Ftr,mC,mFS);';
method{5}.recT='rCqntt=MVBNFrec(Ft,mC,mFS);';
method{5}.THRdef=[1.6 2 3];

%Discriminative mean and variance of K best normalized features
method{6}.name='DMVBKNF';
method{6}.batch='[mC,mCG,mFS]=DMVBKNFbatch(Ftr,Ctr);';
method{6}.recTr='rCqnt=MVBKNFrec(Ftr,mC,mFS);';
method{6}.recT='rCqntt=MVBKNFrec(Ft,mC,mFS);';
method{6}.THRdef=[1.6 2 3];


%Kernel density of best normalised feature - old version
method{7}.name='KDBNF0';
method{7}.batch='[mC,mFS]=KDBNFbatch0(Ftr,Ctr);';
method{7}.recTr='rCqnt=KDBNFrec0(Ftr,mC,mFS);';
method{7}.recT='rCqntt=KDBNFrec0(Ft,mC,mFS);';
method{7}.THRdef=[1.6 2 3];

%Kernel density of best normalised feature
method{8}.name='KDBNF';
method{8}.batch='[mC,mFS]=KDBNFbatch(Ftr,Ctr);';
method{8}.recTr='rCqnt=KDBNFrec(Ftr,mC,mFS);';
method{8}.recT='rCqntt=KDBNFrec(Ft,mC,mFS);';
method{8}.THRdef=[.6 .7 .8];

%Kernel density of best feature
mtd=9;
method{mtd}.name='KDBF';
method{mtd}.batch='[mC,mCG,mFS]=KDBFbatch(Ftr,Ctr);';
method{mtd}.recTr='rCqnt=KDBFrec(Ftr,mC,mFS);';
method{mtd}.recT='rCqntt=KDBFrec(Ft,mC,mFS);';
%method{mtd}.THRdef=[.7 .8 .9];
method{mtd}.THRdef=[.99 .995 .999];

%Mean and variance of best  feature
method{10}.name='MVBF';
method{10}.batch='[mC,mCG,mFS]=MVBFbatch(Ftr,Ctr);';
method{10}.recTr='rCqnt=MVBFrec(Ftr,mC,mFS);';
method{10}.recT='rCqntt=MVBFrec(Ft,mC,mFS);';
method{10}.THRdef=[1.6 2 3];

%Kernel density of best feature
mtd=11;
method{mtd}.name='DKDBF';
method{mtd}.batch='[mC,mCG,mFS]=DKDBFbatch(Ftr,Ctr);';
method{mtd}.recTr='rCqnt=KDBFrec(Ftr,mC,mFS);';
method{mtd}.recT='rCqntt=KDBFrec(Ft,mC,mFS);';
method{mtd}.THRdef=[.7 .8 .9];


if ~exist('THRs') | isempty(THRs)
   THRs=method{MTD}.THRdef;
end;

if length(THRs)==1
   THRs=repmat(THRs,1,3);
end;

%%%%%%%%%%%% LOAD DATA %%%%%%%%%%%%%%%%%%%
if loadData %load new data

   %read/load data at the beginning
   if readObjData
      getAllData;
      filterData;
      load data_11av;
   end

   if readShpData
      load dataShapes300;
   end

   if genRelData
      Nall=100;
      P=genrandPairs(Nall);
      C=assignRel(P);
      F=extRelFeatures(P);
      numC=11;
      Cnames=['TL';'TR';'CT';'FT';'NT';'FF';'OL';'IM';'OR';'NR';'FA'];
      Fnames=['x ';'y ';'dx';'dy';'d '];
      imgIdx=1:Nall*2;
   end



   Nall=size(F,2);

   N=Nall/2;
   N=200;
   Nt=Nall-N;

   trImgs=drand(N,1:Nall);
   restImgs=rest(1:Nall,trImgs);
   tImgs=restImgs;%(drand(Nt,restImgs));

   N=length(trImgs);
   trImgIdx=imgIdx(trImgs);
   Ftr=F(:,trImgs);
   Ctr=C(:,trImgs);
   Ctrsf=lf2sf(Ctr);

   Nt=length(tImgs);
   tImgIdx=imgIdx(tImgs);
   Ft=F(:,tImgs);
   Ct=C(:,tImgs);
   Ctsf=lf2sf(Ct);

end %end load new data

disp(['***** ' method{MTD}.name ' *****']);
disp(['THRs=[ ' num2str(THRs) ' ]']);

%%%%%%%%%%%%%%%%%%%% TRAINING %%%%%%%%%%%%%%%%%%%%%%%%%

if learn
   eval(method{MTD}.batch);
end
%% %%%%%%%%%%%%%%%%%% TEST %%%%%%%%%%%%%%%%%%%%%%%%%

%TEST STAGE ON TRAINING DATA
eval(method{MTD}.recTr);
rCtr=qnt2ql(rCqnt,THRs);

%TEST STAGE ON TEST DATA
eval(method{MTD}.recT);
rCt=qnt2ql(rCqntt,THRs);


%%%%%%%%%%%%%%%%%%%% EVALUATION %%%%%%%%%%%%%%%%%%%%%%%%%

%EVALUATE RESULTS
resCtr=evalRes(rCtr,Ctr);
resCt=evalRes(rCt,Ct);

%print results
printRes(resCtr,'Ctr: ', {'tpf1s','tnf1s'});
printRes(resCt,'Ct : ', {'tpf1s','tnf1s'});

printCG(mCG,Fnames,Cnames);

dfigure('Models');showLmodels(mC,Fnames,Cnames);

%plot results
if showRes
   dfigure(5,1,0);
   plotRes(Ctr,rCtr,resCtr,Cnames,[method{MTD}.name ': Ctr: ']);
   dfigure(5,2,0);
   plotRes(Ct,rCt,resCt,Cnames,[method{MTD}.name ': Ct: ']);
end

%%
return;

%optional:
showROIs(tImgIdx(IDXS),'C:\danijels\Matlab\data\cogLearn\objects\can\');
showROIs(trImgIdx(IDXS),'C:\danijels\Matlab\data\cogLearn\objects\can\');
