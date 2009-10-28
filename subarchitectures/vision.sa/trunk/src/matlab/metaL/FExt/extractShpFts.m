%%
% Originally a part of: CuriousDaniel (developed within EU project CogX)
% Author: Matej Kristan, 2009 (matej.kristan@fri.uni-lj.si; http://vicos.fri.uni-lj.si/matejk/)
% Last revised: 2009
%%
function sout = extractShpFts( X )

if isempty(X)
    sout = [ 0, 0, 0 ] ;
end

C = cov(X) ;
[U,S,V] = svd(C) ;
S = diag(S) ;

% Y = X*U' ;
% ma = max(Y) ;
% mi = min(Y) ;
% Sa = sort(abs(ma - mi), 'descend') ;


sout = [ S(1)*1000, S(2)*1000, S(1) / (S(1)+S(2)) ] ;


