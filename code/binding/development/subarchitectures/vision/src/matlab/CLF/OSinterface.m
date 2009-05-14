function [x,b]=OSinterface

disp(['OSinterface: I will segment the image!']);

%x=getImage;  %call VS
x=VSinterface;

[x,b]=segmentImg(x);