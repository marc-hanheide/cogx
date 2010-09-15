function dispRec

global LRguiR Disp

set(LRguiR,'PaperPosition',[0 0 8 2.5]);
print(LRguiR,Disp.mR,'-dpng', '-r0');