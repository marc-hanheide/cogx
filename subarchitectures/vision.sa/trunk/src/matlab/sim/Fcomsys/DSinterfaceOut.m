function answ=DSinterfaceOut(req)

%DSinterfaceOut
global DSout;
DSout=req;
answ=VMinterface(req);

