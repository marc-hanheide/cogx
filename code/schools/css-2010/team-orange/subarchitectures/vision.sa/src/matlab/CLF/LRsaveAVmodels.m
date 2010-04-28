function LRsaveAVmodels(fname)

global mAV mDA mFS NUs RSs;

%save(fname,'mAV','mFS','mDA');
save(fname,'mAV','mFS','mDA','NUs','RSs');
disp(['AV models saved in ' fname ' .']);
%[numLAV,confs]=checkAV(mAV)
[nlc,nec,LC,EC,confs]=numConcepts(mAV);
fprintf('nlc=%d nec=%d   confs=', nlc, nec);disp(confs);


