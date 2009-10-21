function [x,b]=segmentImg(x);
%b=segmentImg(x);
%Segment image.
%x: RGB image
%b: segmentation mask

xg=rgb2gray(x);
thr=Otsu(xg);
b=xg>thr;
