function CLFquitMO

global Figs;
clfHs=[
   Figs.LRguiL.main
   Figs.LRguiR.main
   Figs.dlgH.main
   Figs.vmcH.main
   Figs.vmipH.main
   Figs.vsHs.main
   Figs.atH.main
];   
close(clfHs);

%close all hidden