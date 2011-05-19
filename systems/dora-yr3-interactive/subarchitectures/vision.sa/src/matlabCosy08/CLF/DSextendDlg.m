function dlg1=DSextendDlg(sentence)

if nargin<1
   sentence='';
end;

global lbdH;
dlg=get(lbdH,'String');
dlg1=[dlg; sentence];
set(lbdH,'String',dlg1);
set(lbdH,'Value',size(dlg1,1));


