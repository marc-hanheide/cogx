function f=FEinterface(x,b)

x=uint8(x);
f=extAPfeatures(x,b);
fprintf('  F= ');fprintf('%3.3g  ',f);fprintf('\n');
showROI(x,b,f);

