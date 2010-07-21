%%
% Originally a part of: Maggot (developed within EU project CogX)
% Author: Matej Kristan, 2009 (matej.kristan@fri.uni-lj.si; http://vicos.fri.uni-lj.si/matejk/)
% Last revised: 2009
%%
function C = regularizeCovariance( C, varargin )

minVal = 1e-10 ;
practicallyZero = [] ;
% process arguments
args = varargin;
nargs = length(args);
for i = 1:2:nargs
    switch args{i}        
        case 'practicallyZero', practicallyZero = args{i+1} ; 
    end
end

if isempty(practicallyZero)
    practicallyZero = 1e-10 ;
end

[U,S,V] = svd(C) ;
s = diag(S) ;

if min(s) < minVal  
   id_ok = ( s > minVal ) ;
   if sum(id_ok) > 0
       defaultval = mean(s( id_ok ))*1e-2 ;
   else
       defaultval = minVal ;
   end
  
   s( s <= minVal ) = defaultval ;  
   
   C = U*diag(s)*U' ;
end