function switchDfigure(str);
%SWITCHDFIGURE enables or disables DFIGURE.
%   SWITCHDFIGURE ON enables creation of new figure windows by DFIGURE.
%   SWITCHDFIGURE OFF disables creation of new figure windows by DFIGURE.
%   SWITCHDFIGURE toggles between ON and OFF.
%
%   See also DINIT, DFIGURE.

global DFIGON;

nin=nargin;

if nin==0
   if DFIGON==0 
      DFIGON=1;
   else
      DFIGON=0;
   end;
else
   if strcmp(str,'off') | strcmp(str,'OFF')
      DFIGON=0;
   else   
      DFIGON=1;
   end;
end;   
