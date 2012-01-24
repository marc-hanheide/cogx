function CLFquit

global Settings Figs

if ~Settings.CAST
   clfHs=[
      Figs.LRguiL.main
      Figs.LRguiR.main
      Figs.dlgH.main
      Figs.vmcH.main
      Figs.vmipH.main
      Figs.vsHs.main
      Figs.atH.main
      ];
else
   clfHs=[
      Figs.LRguiL.main
      Figs.LRguiR.main
      ];
end

close(clfHs);
