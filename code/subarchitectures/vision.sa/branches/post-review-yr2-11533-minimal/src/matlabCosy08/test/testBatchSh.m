%testBatchSh
%test bach learing of concepts
%real shape images

%set switches
readData=0;
loadData=1;
setData=1;
showRes=1;
learn=1;


%% %%%%%%%%%% PREPARE DATA %%%%%%%%%%%%%%%%%%%

if readData
   getAllDataSh;
   filterDataSh;
   Fnames=['Hu';'Sa';'In';'S1';'S2';'S3';'S4'];
   CTT=[1:7;1 1 1 1 2 2 2]';
end

if loadData
%   load dataShapes300
%   load dataShapesBHAM199
   load dataObjects296
end;

if setData %re-set training and test images

   Nall=size(F,2);

   N=Nall/2; %number of training images

   trImgs=drand(N,1:Nall);
   restImgs=rest(1:Nall,trImgs);
   tImgs=restImgs;%(drand(Nt,restImgs));

   %training images
   N=length(trImgs);
   trImgIdx=imgIdx(trImgs);
   Ftr=F(:,trImgs);
   Ctr=C(:,trImgs);
   Ctrsf=lf2sf(Ctr);

   %test images
   Nt=length(tImgs);
   tImgIdx=imgIdx(tImgs);
   Ft=F(:,tImgs);
   Ct=C(:,tImgs);
   Ctsf=lf2sf(Ct);
   
   Ftr=Ftr+rand(size(Ftr))*1e-2;

end

%% SHOW DISTRIBUTIONS OF FEATURES
if showRes

   dfigure('Dist F, colours');
   for j=1:4
      indxs{j}=find(C(j,:));
   end
   allindxs=unique([indxs{1} indxs{2} indxs{3} indxs{4}]);
   indxs{5}=find(~ismember(1:Nall,allindxs));
   clrs='rgbykcm';
   for i=1:7
      subplot(2,4,i);
      title(Fnames(i,:));
      hold on;
      for j=1:5
         plot(indxs{j},F(i,indxs{j}),[clrs(j) '*']);
      end;
      legend(Cnames(1:4,:));
   end;

   dfigure('Dist F, shapes');
   for j=5:7
      indxs{j}=find(C(j,:));
   end;
   allindxs=unique([indxs{5} indxs{6} indxs{7}]);
   indxs{8}=find(~ismember(1:Nall,allindxs));
   clrs='rgbykcmr';
   for i=1:7
      subplot(2,4,i);
      title(Fnames(i,:));
      hold on;
      for j=5:8
         plot(indxs{j},F(i,indxs{j}),[clrs(j) '*']);
      end;
      legend(Cnames(5:7,:));
   end;

end


%% %%%%%%%%%%%%%%%%%% TRAINING %%%%%%%%%%%%%%%%%%%%%%%%%

if learn
   [mC,mCG,mFS]=KDBFbatch(Ftr,Ctr);
   printCG(mCG,Fnames,Cnames);
   dfigure('Models');showLmodels(mC,Fnames,Cnames);
end

%% %%%%%%%%%%%%%%%%%% TEST REC %%%%%%%%%%%%%%%%%%%%%%%%%

THRs=[1 1 1]/100;
disp(['*****  TEST REC *****']);
disp(['THRs=[ ' num2str(THRs) ' ]']);

%TEST STAGE ON TRAINING DATA
rCqnt=KDBFrec(Ftr,mC,mFS);
rCtr=qnt2ql(rCqnt,THRs);

%TEST STAGE ON TEST DATA
rCqntt=KDBFrec(Ft,mC,mFS);
rCt=qnt2ql(rCqntt,THRs);

% %%%%%%EVALUATION%%%%%%%%

%EVALUATE RESULTS
resCtr=evalRes(rCtr,Ctr);
resCt=evalRes(rCt,Ct);

%print results
printRes(resCtr,'Ctr: ', {'tpf1s','tnf1s'});
printRes(resCt,'Ct : ', {'tpf1s','tnf1s'});

%plot results
if showRes
   dfigure(5,4,0);
   plotRes(Ctr,rCtr,resCtr,Cnames,'Ctr: ');
   plotRes(Ct,rCt,resCt,Cnames,'Ct: ');
end


%% %%%%%%%%%%%%%%%%%% TEST DISCR %%%%%%%%%%%%%%%%%%%%%%%%%

THRs=[1 1 1]*1e-12;
disp(['*****  TEST RECD *****']);
disp(['THRs=[ ' num2str(THRs) ' ]']);

%TEST STAGE ON TRAINING DATA
pcx=KDBFDrecSh(Ftr,mC,mFS);
rCtr=qnt2qlDsh(pcx,THRs,CTT);

%TEST STAGE ON TEST DATA
pcxt=KDBFDrecSh(Ft,mC,mFS);
rCt=qnt2qlDsh(pcxt,THRs,CTT);


% %%%%%%EVALUATION%%%%%%%%

%EVALUATE RESULTS
resCtr=evalRes(rCtr,Ctr);
resCt=evalRes(rCt,Ct);

%print results
printRes(resCtr,'Ctr: ', {'tpf1s','tnf1s'});
printRes(resCt,'Ct : ', {'tpf1s','tnf1s'});

%plot results
if showRes
   dfigure(5,2,0);
   plotRes(Ctr,rCtr,resCtr,Cnames,'Ctr: ');
   plotRes(Ct,rCt,resCt,Cnames,'Ct: ');
end

%%

return;

%optional:
showROIs(tImgIdx(IDXS),'C:\danijels\Matlab\cogLearn\data\shapesBHAM\');
showROIs(trImgIdx(IDXS),'C:\danijels\Matlab\cogLearn\data\shapesBHAM\');


[42 43 47 49 52 54 58 130 133 137]
imgIdx=[1:41 44:46 48 50 51 53 55:57 59:129 131 132 134:136 138:199];

