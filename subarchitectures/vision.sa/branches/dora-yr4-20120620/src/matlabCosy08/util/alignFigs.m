function alignFigs(figs,pos);
%ALIGNFIGS  Align figures.
%   ALIGNFIGS(FIGS) aligns all figures with figure numbers listed in the row 
%   vector FIGS to match the upper right corner of the current figure.
%   ALIGNFIGS aligns all figures.
%   ALIGNFIGS(FIGS,Pos) places figures in the specified location:
%        0 = Aligned with the current figure (default)
%        1 = Upper right-hand corner of the screen
%        2 = Upper left-hand corner of the screen
%        3 = Lower left-hand corner of the screen
%        4 = Lower right-hand corner of the screen
%
%   See also DINIT, DFIGURE, SHOWFIGS, HIDEFIGS, CLOSEFIGS, COMPOUNDFIGS.

nin=nargin;
if nin==0 %align all figures
   numfigs=figure;
   close(numfigs);
   numfigs=numfigs-1;
   figs=[1:numfigs];
end;   

if nin<2 pos=0; end;

global DFIGTBW DFIGBW;

nfigs=length(figs);
for i=1:nfigs
   posi=get(figs(i),'Position');
   switch pos
   case 0
		pos1=get(gcf,'Position');
      xi=pos1(1)+pos1(3)-posi(3);
      yi=pos1(2)+pos1(4)-posi(4);
   case 1
		screen = get(0, 'ScreenSize');
      xi=screen(3)-posi(3)-DFIGBW;
      yi=screen(4)-posi(4)-DFIGTBW;
   case 2
		screen = get(0, 'ScreenSize');
      xi=DFIGBW;
      yi=screen(4)-posi(4)-DFIGTBW;
   case 3
		screen = get(0, 'ScreenSize');
      xi=DFIGBW;
      yi=DFIGBW;
   case 4
		screen = get(0, 'ScreenSize');
      xi=screen(3)-posi(3)-DFIGBW;
      yi=DFIGBW;
   end;   
   
   %set(figs(i),'Position',[pos1(1:2)+pos1(3:4)-posi(3:4) posi(3:4)]);
   set(figs(i),'Position',[xi yi posi(3:4)]);
end;

showFigs(figs);