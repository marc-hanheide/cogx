function displayG(hp,type)

global Disp Settings

switch type
   
   case 'GL'
      if Settings.Disp.GL
         set(hp,'PaperPosition',[0 0 8 4.3]);
         print(hp,Disp.mGL,'-dpng', '-r0');
      end
      
   case 'GR'
      
      if Settings.Disp.GR
         set(hp,'PaperPosition',[0 0 8 2.5]);
         print(hp,Disp.mGR,'-dpng', '-r0');
      end
      
   case 'GD'
      if Settings.Disp.GD
         set(hp,'PaperPosition',[0 0 8 2.5]);
         print(hp,Disp.mGD,'-dpng', '-r0');
      end
      
end
