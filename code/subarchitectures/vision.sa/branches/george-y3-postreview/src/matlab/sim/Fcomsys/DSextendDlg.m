function dlg1=DSextendDlg(sentence)

if nargin<1
   sentence='';
end;

global Figs
dlg=get(Figs.dlgH.lbdH,'String');
dlg1=[dlg; sentence];
set(Figs.dlgH.lbdH,'String',dlg1);
set(Figs.dlgH.lbdH,'Value',size(dlg1,1));


