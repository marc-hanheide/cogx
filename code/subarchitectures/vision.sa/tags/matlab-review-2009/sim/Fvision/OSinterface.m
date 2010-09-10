function [x,b, pt3d]=OSinterface

disp(['OSinterface: I will segment the image!']);

%x=getImage;  %call VS
[x,b, pt3d]=VSinterface;

%[x,b]=segmentImg(x);