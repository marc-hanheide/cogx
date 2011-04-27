function VSstart

%global Settings
global Figs


%Settings.Params.currIdx=0;

vsHs=VScontrol;

Figs.vsHs.main=vsHs;

Figs.vsHs.axCimgH=findobj(vsHs, 'Tag','axCurrImg');
Figs.vsHs.axCpts3dH=findobj(vsHs, 'Tag','axCurr3d');

f1=figure;
 set(f1,'Visible','off');
VSnextImg;
%close(f1);



