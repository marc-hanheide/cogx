function LRsaveAVmodels(fname)

global mC

save(fname,'mC');
disp(['Models saved in ' fname ' .']);

[nlc,nec,LC,EC,confs]=numConcepts(mC);
fprintf('nlc=%d nec=%d   confs=', nlc, nec);disp(confs);


