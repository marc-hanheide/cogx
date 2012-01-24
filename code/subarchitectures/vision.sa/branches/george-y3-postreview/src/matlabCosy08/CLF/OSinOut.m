function [x,b]=OSinOut

x=getImage;  %call VS

[x,b]=segmentImg(x);