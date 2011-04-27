function m=montageFigs(figs);
%MONTAGEFIGS  Montage figures.
%   MONTAGEFIGS(FIGS) montages images in figures FIGS into one figure. 
%   The images have to be of equal size.
%
%   See also DFIGURE, CLOSEFIGS, COMPOUNDFIGS.

nfigs=length(figs);
for i=1:nfigs
   oh=findobj(figs(i),'type','image');
   img1=get(oh,'CData');
   scmin=min(min(img1));
   scmax=max(max(img1));
   img(:,:,1,i)=(img1-scmin)*255/(scmax-scmin);
end;
m=montage(uint8(img));
