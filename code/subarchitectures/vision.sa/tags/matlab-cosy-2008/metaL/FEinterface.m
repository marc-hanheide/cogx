function f=FEinterface(x,b)

global Params

x=uint8(x);
f=extAPfeatures(x,b,Params.FV);
fprintf('  F= ');fprintf('%3.3g  ',f);fprintf('\n');
showROI(x,b,f);

