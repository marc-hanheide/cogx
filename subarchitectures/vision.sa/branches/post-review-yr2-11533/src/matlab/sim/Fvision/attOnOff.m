function atOn=attOnOff

global Settings

if get(Figs.atH.attOnH,'Value')==1
   Settings.attOn=1;
else
   Settings.attOn=0;
end;

atOn=attOn;
