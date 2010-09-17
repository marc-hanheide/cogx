function VMstart

global currState Figs
currState=struct('f',[],'lastRansw',[]);
%currMode=struct('learnMode',2,'THRs',[2 4 5],'wT',1,'wYes',.75,'wPy',.25);
%currMode=struct('learnMode',2,'THRs',[.9 .95 .98],'wT',1,'wYes',.75,'wPy',.25);

Figs.vmcH.main=VMcontrol;
VMintProcStart;

%vmHs=[vmcH vmipH];
%vmHs=[vmipH vmipH];