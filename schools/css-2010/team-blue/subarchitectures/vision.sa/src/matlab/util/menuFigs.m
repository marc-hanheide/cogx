function menuFigs(figs);
%MENUFIGS  Show menu in figures.
%  MENUFIGS(figs) shows the menu in figures with figure numbers listed in 
%  the row vector figs.
%
%  MENUFIGS shows the menu in the current figure.
%
%  See also DINIT, DFIGURE, CLOSEFIGS, SHOWFIGS, COMPOUNDFIGS.


nin=nargin;
if nin==0 %only current figure
    set(gcf,'menubar','figure');  
else
    nfigs=length(figs);
    for i=1:nfigs
        set(figs(i),'menubar','figure');
    end;
end