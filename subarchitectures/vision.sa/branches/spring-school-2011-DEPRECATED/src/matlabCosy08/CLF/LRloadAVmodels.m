function LRloadAVmodels(fname)

global mAV mDA mFS NUs RSs;

load(fname);

disp(['AV models loaded from ' fname ' .']);
%[numLAV,confs]=checkAV(mAV)
[nlc,nec,LC,EC,confs]=numConcepts(mAV);
fprintf('nlc=%d nec=%d   confs=', nlc, nec);disp(confs);

LRvisUpdate;
%LRevalUpdate;

