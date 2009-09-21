function closeFigs(figs);
%CLOSEFIGS  Close figures.
%   CLOSEFIGS(FIGS) closes figures with figure numbers listed in FIGS.
%   CLOSEFIGS closes all figures.
%
%   See also DINIT, DFIGURE, SHOWFIGS, COMPOUNDFIGS.

nin=nargin;
if nin==0 %close all
   close all
else
   nfigs=length(figs);
   for i=1:nfigs
      close(figs(i));
   end;
end;   