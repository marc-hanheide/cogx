function displayG(hp,type)

global Disp Settings

switch type
   
   case 'GL'
      if Settings.Disp.GL
         displayG_renderToFile(hp, Disp.mGL);
      end
      
   case 'GR'
      
      if Settings.Disp.GR
         displayG_renderToFile(hp, Disp.mGR);
      end
      
   case 'GD'
      if Settings.Disp.GD
         displayG_renderToFile(hp, Disp.mGD);
      end
      
end

% This function will render a figure to a file.
% A temporary file must be created and copied so that the
% CogX/CAST DisplayServer can pick up the final file with the FileMonitor.
%
% The FileMonitor triggers an event when a file is closed afer it has
% been modified. Unfortunately Matlab opens and closes the image it is
% generating 3 times leaving it in an invalid state in the meantime.
% If the FileMonitor picks up an invalid file, this crashes the DisplayServer.
% Therefore the figure has to be rendered to a temporary file and 
% has to be _copied_ to a new location. Moving a file doesn't generate
% an event for the FileMonitor (unless the temporary file is on a different
% file system than the final file, which is not the case).
function displayG_renderToFile(hp, fname)
   global Disp Settings
   set(hp, 'PaperPositionMode', 'auto');
   dpi = ['-r' num2str(Settings.Disp.printDpi)];
   ftmp = [fname '_tmp'];
   print(hp, ftmp, '-noui', '-dpng', dpi);
   copyfile(ftmp, fname);
   delete(ftmp);
