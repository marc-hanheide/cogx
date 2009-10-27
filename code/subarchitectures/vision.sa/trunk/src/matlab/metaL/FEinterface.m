function f=FEinterface(x,b,pt3d)

global Params

x=uint8(x);
f=extAPfeatures(x,b,Params.FV,pt3d);
fprintf('  F= ');fprintf('%3.3g  ',f);fprintf('\n');
showROI(x,b,f);

