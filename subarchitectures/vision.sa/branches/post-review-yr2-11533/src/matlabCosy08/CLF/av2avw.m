function avw=av2avw(av,w)

if nargin<2 
   w=1;
end;   

avw=[av',ones(length(av)',1)*w];