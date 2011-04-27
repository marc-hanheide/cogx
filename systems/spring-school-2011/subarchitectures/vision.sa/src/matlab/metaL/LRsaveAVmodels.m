function LRsaveAVmodels(fname)

global mC

%save(fname,'mC','mFS','mDA');
%save(fname,'mC','mFS','mDA','NUs','RSs');
save(fname,'mC');
disp(['AV models saved in ' fname ' .']);
%[numLAV,confs]=checkAV(mC)
[nlc,nec,LC,EC,confs]=numConcepts(mC);
fprintf('nlc=%d nec=%d   confs=', nlc, nec);disp(confs);


