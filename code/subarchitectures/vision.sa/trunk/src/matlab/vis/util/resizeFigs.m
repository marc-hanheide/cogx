function resizeFigs(figs,resw,resh);
%RESIZEFIGS  Resize figures.
%   RESIZEFIGS(FIGS) resizes figures with figure numbers listed in the row vector 
%   FIGS by factor 2.
%
%   RESIZEFIGS(FIGS,RESW) resizes figures by factor RESW.
%
%   RESIZEFIGS(FIGS,RESW,RESH) - figure width is resized by factor RESW, 
%   figure height is resized by factor RESH.
%
%   See also DFIGURE, CLOSEFIGS.

nin=nargin;
if nin==1 resw=2; end;

nfigs=length(figs);
for i=1:nfigs
   p=get(figs(i),'Position');
   ix=p(1);iy=p(2);iw=p(3);ih=p(4);
   if resw<=10
      niw=round(iw*resw);
      if nin<3 resh=resw; end;
      nih=round(ih*resh);
   else
      niw=resw;
      if nin<3 resh=round(niw*ih/iw); end;
      nih=resh;
   end;          
   set(figs(i),'Position',[ix,iy,niw,nih]);
end;
