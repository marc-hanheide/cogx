function [hu,sa,in]=appFeatures(x,b)
%[hu,sa,in]=appFeatures(x,b)
%Calculate appearance features.
%x: one RGB image in (IWH,IWH,3) form
%b: binary segmentation mask of size (IWH,IWH)
%hu, sa, in: feature values (scalars - medians of hue, saturation and value
%of foreground pixels.


IH=size(x,1);
IW=size(x,2);

%transform to HSV
HSV=rgb2hsv(x);
HSV(HSV(:,:,1)>.8)=0; %red color...
%idxs=find(HSV(:,:,1)>.8);HSV(idxs)=1-HSV(idxs); %red color...

%consider foreground only
idxs=find(b==1);
V0=reshape(HSV,IH*IW,1,3);
V=V0(idxs,:);

%get median
medHSV=median(V)';
[hu,sa,in]=deal(medHSV(1),medHSV(2),medHSV(3));


