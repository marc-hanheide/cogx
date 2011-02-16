function sompak_sammon_gui()

%SOMPAK_SAMMON_GUI A GUI for using SOM_PAK Sammon's mapping program 
%                  from Matlab.
%
%  sompak_sammon_gui
%
% Launches a GUI which allows the use of SOM_PAK Sammon's mapping
% program (sammon) from Matlab. Notice that to use this function, the
% SOM_PAK programs must be in your search path, or the variable
% 'SOM_PAKDIR' which is a string containing the program path, must be
% defined in the workspace. SOM_PAK programs can be found from:
% http://www.cis.hut.fi/research/som_lvq_pak.shtml
%
% See also SOMPAK_SAMMON, SOMPAK_GUI, SOMPAK_INIT_GUI,
%          SOMPAK_TRAIN_GUI, SOM_GUI.

% Contributed to SOM Toolbox vs2, February 2nd, 2000 by Juha Parhankangas
% Copyright (c) by Juha Parhankangas
% http://www.cis.hut.fi/projects/somtoolbox/

% Juha Parhankangas 050100

h=findobj(get(0,'Children'),'Tag','SammonGUI');

if ~isempty(h)
  figure(h);
  return;
end


a = figure('Color',[0.8 0.8 0.8], ...
	'PaperType','a4letter', ...
	'Position',[665 517 175 295], ...
	'Tag','SammonGUI');
b = uicontrol('Parent',a, ...
	'Units','points', ...
	'BackgroundColor',[0.701961 0.701961 0.701961], ...
	'Callback','close gcf', ...
	'FontWeight','demi', ...
	'Position',[8 20 50 20], ...
	'String','CLOSE', ...
	'Tag','Pushbutton1');
b = uicontrol('Parent',a, ...
	'Units','points', ...
	'BackgroundColor',[0.701961 0.701961 0.701961], ...
	'Callback','sompak_rb_control sammon_ok',...
	'FontWeight','demi', ...
	'Position',[86 20 50 20], ...
	'String','OK', ...
	'Tag','Pushbutton2');
b = uicontrol('Parent',a, ...
	'Units','points', ...
	'BackgroundColor',[0.701961 0.701961 0.701961], ...
	'Position',[8 50 130 175], ...
	'Style','frame', ...
	'Tag','Frame1');
b = uicontrol('Parent',a, ...
	'Units','points', ...
	'BackgroundColor',[0.8 0.8 0.8], ...
	'Position',[12 54 122 40], ...
	'Style','frame', ...
	'Tag','Frame2');
b = uicontrol('Parent',a, ...
	'Units','points', ...
	'BackgroundColor',[0.8 0.8 0.8], ...
	'FontWeight','demi', ...
	'HorizontalAlignment','left', ...
	'Position',[30 78 90 12], ...
	'String','RUNNING LENGTH', ...
	'Style','text', ...
	'Tag','StaticText1');
b = uicontrol('Parent',a, ...
	'Units','points', ...
	'BackgroundColor',[1 1 1], ...
	'Callback','sompak_rb_control rlen',...
	'Position',[48 57 50 20], ...
	'Style','edit', ...
	'Tag','RLEN');

udata.rlen=[];

b = uicontrol('Parent',a, ...
	'Units','points', ...
	'BackgroundColor',[0.8 0.8 0.8], ...
	'Position',[12 96 122 40], ...
	'Style','frame', ...
	'Tag','Frame3');
b = uicontrol('Parent',a, ...
	'Units','points', ...
	'BackgroundColor',[0.8 0.8 0.8], ...
	'FontWeight','demi', ...
	'HorizontalAlignment','left', ...
	'Position',[33 119 90 12], ...
	'String','OUTPUT VARIABLE', ...
	'Style','text', ...
	'Tag','StaticText2');
b = uicontrol('Parent',a, ...
	'Units','points', ...
	'BackgroundColor',[1 1 1], ...	
	'Callback','sompak_rb_control out_var',...
	'Position',[48 99 50 20], ...
	'String','''ans''', ...
	'Style','edit', ...
	'Tag','OUT_VAR');

udata.out_var='ans';

b = uicontrol('Parent',a, ...
	'Units','points', ...
	'BackgroundColor',[0.8 0.8 0.8], ...
	'Position',[12 138 122 40], ...
	'Style','frame', ...
	'Tag','Frame4');
b = uicontrol('Parent',a, ...
	'Units','points', ...
	'BackgroundColor',[0.8 0.8 0.8], ...
	'FontWeight','demi', ...
	'HorizontalAlignment','left', ...
	'Position',[43 162 60 12], ...
	'String','OUTPUT FILE', ...
	'Style','text', ...
	'Tag','StaticText3');
b = uicontrol('Parent',a, ...
	'Units','points', ...
	'BackgroundColor',[1 1 1], ...
	'Callback','sompak_rb_control out_file',...
	'Position',[15 141 50 20], ...
	'Style','edit', ...
	'Tag','OUT_FILE');

udata.out_file=[];

b = uicontrol('Parent',a, ...
	'Units','points', ...
	'Callback','sompak_rb_control out_ft',...
	'FontWeight','demi', ...
	'HorizontalAlignment','left', ...
	'Max',3, ...
	'Min',1, ...
	'Position',[70 146 62 15], ...
	'String',{'No File';'mat-file';'cod-file'}, ...
	'Style','popupmenu', ...
	'Tag','OUT_FILE_TYPE', ...
	'Value',1);

udata.out_file_type='';

b = uicontrol('Parent',a, ...
	'Units','points', ...
	'BackgroundColor',[0.8 0.8 0.8], ...
	'Position',[12 180 122 40], ...
	'Style','frame', ...
	'Tag','Frame5');
b = uicontrol('Parent',a, ...
	'Units','points', ...
	'BackgroundColor',[0.8 0.8 0.8], ...
	'FontWeight','demi', ...
	'HorizontalAlignment','left', ...
	'Position',[60 203 25 12], ...
	'String','MAP', ...
	'Style','text', ...
	'Tag','StaticText4');
b = uicontrol('Parent',a, ...
	'Units','points', ...
	'BackgroundColor',[1 1 1], ...
	'Callback','sompak_rb_control map',...
	'Position',[15 183 50 20], ...
	'Style','edit', ...
	'Tag','MAP');

udata.map=[];

b = uicontrol('Parent',a, ...
	'Units','points', ...
	'Callback','sompak_rb_control map_ft',... 
	'FontWeight','demi', ...
	'HorizontalAlignment','left', ...
	'Max',3, ...
	'Min',1, ...
	'Position',[70 188 62 15], ...
	'String',{'Variable';'mat-file';'cod-file'}, ...
	'Style','popupmenu', ...
	'Tag','MAP_TYPE', ...
	'Value',1);

udata.map_type='';

b = uicontrol('Parent',a, ...
	'Units','points', ...
	'BackgroundColor',[0.8 0.8 0.8], ...
	'FontSize',12, ...
	'FontWeight','demi', ...
	'Position',[41 230 62 12], ...
	'String','SAMMON', ...
	'Style','text', ...
	'Tag','StaticText5');


set(gcf,'UserData',udata);
