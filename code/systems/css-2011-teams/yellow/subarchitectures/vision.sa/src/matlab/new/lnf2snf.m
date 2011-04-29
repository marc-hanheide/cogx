function C=lnf2snf(CC)
%function CC=c2cc(C,CM)
%From new format CC to old ordered long format C

numSC=length(CC);
N=size(CC{1},2);
C=zeros(numSC,N);

numc=0;
for sc=1:numSC
   Csc=lf2sf(CC{sc}); 
   idxs=find(Csc>0);
   %Csc(idxs)=Csc(idxs)-numc;
   C(sc,:)=Csc;
   numc=numc+size(CC{sc},1);
end
