function LRloadAVmodels(fname)

global mC

load(fname);

disp(['AV models loaded from ' fname ' .']);
%[numLAV,confs]=checkAV(mC)
%[nlc,nec,LC,EC,confs]=numConcepts(mC);
%fprintf('nlc=%d nec=%d   confs=', nlc, nec);disp(confs);
getc(mC,'info')

LRvisUpdate;
%LRevalUpdate;

