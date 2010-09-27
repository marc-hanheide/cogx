function displayG(hp,type)

global Disp Settings

switch type
   
   case 'GL'
      if Settings.Disp.GL
         set(hp,'PaperPositionMode', 'auto');
         dpi = ['-r' num2str(Settings.Disp.printDpi)];
         print(hp, Disp.mGL, '-noui', '-dpng', dpi);
      end
      
   case 'GR'
      
      if Settings.Disp.GR
         set(hp, 'PaperPositionMode', 'auto');
         dpi = ['-r' num2str(Settings.Disp.printDpi)];
         print(hp, Disp.mGR, '-noui', '-dpng', dpi);
      end
      
   case 'GD'
      if Settings.Disp.GD
         set(hp,'PaperPositionMode', 'auto');
         dpi = ['-r' num2str(Settings.Disp.printDpi)];
         print(hp, Disp.mGD, '-noui', '-dpng', dpi);
      end
      
end
