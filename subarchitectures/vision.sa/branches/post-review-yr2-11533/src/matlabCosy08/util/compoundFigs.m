function compoundFigs(figs,ai);
%COMPOUNDFIGS  Compound figures.
%   COMPOUNDFIGS(FIGS) compounds figures with figure numbers listed in 
%   the row vector FIGS into a new figure.
%   COMPOUNDFIGS(FIGS,AI) compounds AI figures in one row.
%
%   See also DFIGURE, DINIT, CLOSEFIGS.

nfigs=length(figs);
nin=nargin;
if nin==1
   ai=ceil(sqrt(nfigs));
end;
aj=ceil(nfigs/ai);

cf=gcf;
for i=1:nfigs
   tah=subplot(aj,ai,i);
   tpos=get(tah,'Position');
   delete(tah);
   if figs(i)>0
   	ah=findobj(figs(i),'type','axes');
	   nah=copyobj(ah,cf);
      set(nah,'Position',tpos);
   end;   
end;
