function answ=DSinterface(req)

%DSinterfaceOut
global DSout;
DSout=req;
answ=VMinterface(req);

return

%DSinterfaceIn
global txrH;
disp(['DSinterface: I received [' num2str(answ) ']!']);
sentence=DScreateSentence(answ);
set(txrH,'String',sentence);

DSextendDlg(['R: ' sentence]);
