function dispLearn(hp)

global Disp Settings

if Settings.Disp.GL   
   set(hp,'PaperPosition',[0 0 8 4.3]);
   print(hp,Disp.mGL,'-dpng', '-r0');
end