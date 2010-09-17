function DSinterfaceIn(answ)

global Figs
disp(['DSinterfaceIn: I received [' num2str(answ) ']!']);
sentence=DScreateSentence(answ);
set(Figs.dlgH.txrH,'String',sentence);

DSextendDlg(['R: ' sentence]);
