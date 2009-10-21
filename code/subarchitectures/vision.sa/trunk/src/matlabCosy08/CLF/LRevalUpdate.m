function LRevalUpdate

%return;

global Ft AVt mAV mFS currMode ;

numAV=6;

%unc=MVBFrec(Ft,mAV,mFS);
unc=KDBFrec(Ft,mAV,mFS);
rAV=qnt2ql(unc,currMode.THRs);
rAVc=lf2lof(rAV,numAV);
RR=evalRes(rAVc,AVt);
rs=RR.rs;

%[avs,confs]=checkAV(mAV);
[foo,foo,foo,foo,confs]=numConcepts(mAV);
wnu=sum(confs);

global NUs RSs; 

RSs=[RSs rs];
NUs=[NUs wnu];

global LRevalH;

showEval(NUs,RSs,LRevalH.ax_RS);