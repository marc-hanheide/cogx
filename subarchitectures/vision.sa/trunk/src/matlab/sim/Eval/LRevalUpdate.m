function LRevalUpdate

%return;

global Ft AVt mC Params

numC=6;

%unc=MVBFrec(Ft,mC,mFS);
%unc=KDBFrec(Ft,mC,mFS);
unc=MKDBFrec(Ft,mC);
rAV=qnt2ql(unc,Params.THRs);
rAVc=lf2lof(rAV,numC);
RR=evalRes(rAVc,AVt);
rs=RR.rs;

%[avs,confs]=checkAV(mC);
[foo,foo,foo,foo,confs]=numConcepts(mC);
wnu=sum(confs);

global NUs RSs; 

RSs=[RSs rs];
NUs=[NUs wnu];

global LRevalH;

showEval(NUs,RSs,LRevalH.ax_RS);