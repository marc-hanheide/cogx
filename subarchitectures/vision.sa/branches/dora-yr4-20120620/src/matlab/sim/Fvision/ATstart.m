function ATstart

global Figs
global Settings 

atH=ATcontrol;
Figs.atH.main=atH;

Figs.atH.attOnH=findobj(atH, 'tag','rb_attOn');
Figs.atH.attOffH=findobj(atH, 'tag','rb_attOff');

set(Figs.atH.attOnH,'Value',1);
set(Figs.atH.attOffH,'Value',0);
Settings.attOn=1;