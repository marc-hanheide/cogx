%testBatch5
%Ftr,Ctr,Ft,Ct,THRst,MTD,resEER are assumed to be set - use cmlSim

MTD=6;

disp(['******************************* ' MTDname ' **************************************']);


if ~iscell(Ctr)
   CM=[1:6;1 1 1 1 2 2]';
   Ctr=c2cc(Ctr,CM);
end;

%learning
tic;
mC=ODKDEbatch(Ftr,Ctr);
secs=toc;
disp(['Learning time [s]:  ',num2str(secs)]);
disp(['Num. of components: ',num2str(getc(mC,'numComp'))]);

%testing
rCCpcx=ODKDErec(Ft,mC);
rCpcx=cc2c(rCCpcx,'trim');
rC=qnt2ql(rCpcx,THRst);
%rC=lf2lof(rC,numC);

%evaluate the results
resC=evalRes(rC,Ct);
%rt=calcRT(resC.rs,tnumq);
%ng=MTDnumComp(mC);

%Show results
printRes(resC,[MTDname,':  '],{'tpf1s','tnf1s'});
figRes=dfigure(6,1,[MTDname ' results']);
resizeFigs(figRes,6,1);
plotRes1(Ct,rC,resC,Cnames,[MTDname ': ']);


disp(['*** ',MTDname,' ***']);
mCG=[];mFS=0;
MTDdispModel(mC,mCG,Fnames,Cnames);
%printCG(mCG,Fnames,Cnames);

%optimal results
if resEER %calulate results at Equal Error Rate
   
   %disp('*** Results at EER ***');
   
   [eer,thr]=findEER(mC,mFS,Ft,Ct,[0 1]);
   disp(['THR at EER: ' num2str(thr)]);
   
   rCqnt=MTDrec(Ft,mC,mFS);
   rC=qnt2ql(rCqnt,[1 1 1]*thr);
   rC=lf2lof(rC,numC);
   resC=evalRes(rC,Ct);
   
   printRes(resC,[MTDname '@EER:'],{'tpf1s','tnf1s'});
   figResEER=dfigure(6,2,[MTDname '@EER']);
   resizeFigs(figResEER,6,1);
   plotRes1(Ct,rC,resC,Cnames,[LMname ': ']);
end