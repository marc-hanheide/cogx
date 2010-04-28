function hideFigs(figs);
%HIDEFIGS  Hide figures.
%   HIDEFIGS(FIGS) hides figures with figure numbers listed in row vector FIGS.
%   HIDEFIGS hides all figures.
%
%   See also DINIT, DFIGURE, CLOSEFIGS, SHOWFIGS, COMPOUNDFIGS.

nin=nargin;
if nin==0 %hide all figures
   numfigs=figure;
   close(numfigs);
   numfigs=numfigs-1;
   figs=[1:numfigs];
end;   

nfigs=length(figs);
for i=1:nfigs
   set(figs(i),'Visible','off');
end;
