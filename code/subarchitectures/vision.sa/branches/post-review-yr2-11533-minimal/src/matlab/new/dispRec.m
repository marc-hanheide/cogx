function dispRec

global Figs Disp Settings

if Settings.Disp.GR
   set(Figs.LRguiR.main,'PaperPosition',[0 0 8 2.5]);
   print(Figs.LRguiR.main,Disp.mGR,'-dpng', '-r0');
end