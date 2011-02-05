function showFigs(figs);
%SHOWFIGS  Show figures.
%   SHOWFIGS(FIGS) shows figures with figure numbers listed in the row vector FIGS.
%   SHOWFIGS shows all figures.
%
%   See also DINIT, DFIGURE, HIDEFIGS, CLOSEFIGS, COMPOUNDFIGS.

nin=nargin;
if nin==0 %show all figures
   numfigs=figure;
   close(numfigs);
   numfigs=numfigs-1;
   figs=[1:numfigs];
end;   

nfigs=length(figs);
for i=1:nfigs
   figure(figs(i));
end;
