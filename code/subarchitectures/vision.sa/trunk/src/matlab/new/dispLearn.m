function dispLearn(hp)

global LRguiL Disp

set(hp,'PaperPosition',[0 0 8 4.3]);
print(hp,Disp.mL,'-dpng', '-r0');