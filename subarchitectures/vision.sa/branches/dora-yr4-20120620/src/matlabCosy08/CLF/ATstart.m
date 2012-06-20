function atH=ATstart

global attOn attOnH attOffH;

atH=ATcontrol;

attOnH=findobj(atH, 'tag','rb_attOn');
attOffH=findobj(atH, 'tag','rb_attOff');

set(attOnH,'Value',1);
set(attOffH,'Value',0);
attOn=1;