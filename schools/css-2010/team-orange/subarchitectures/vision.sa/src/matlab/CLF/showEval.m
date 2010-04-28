function showEval(NUs,RSs,ax)

% set(ax,'PlotBoxAspectRatioMode','auto', ...
%    'DataAspectRatioMode'   ,'auto', ...
%    'CameraViewAngleMode'   ,'auto');
% 

plot(NUs,RSs,'.b-','Parent',ax);
set(get(ax,'XLabel'),'String','weighted number of updates');
set(get(ax,'YLabel'),'String','recognition score');
set(ax,'Color','none');
set(ax,'Box','on');
