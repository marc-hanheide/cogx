function DSinterfaceIn(answ)

global txrH;
disp(['DSinterfaceIn: I received [' num2str(answ) ']!']);
sentence=DScreateSentence(answ);
set(txrH,'String',sentence);

DSextendDlg(['R: ' sentence]);
