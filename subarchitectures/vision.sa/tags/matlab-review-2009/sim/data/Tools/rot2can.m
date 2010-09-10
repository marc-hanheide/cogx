function [x,b]=rot2can(x0,b0)


b1=bin(b0);
[b2,bb2]=crop(b1);
x2=crop(x0,bb2);

rp=regionprops(bwlabel(b2),'orientation');

b3=imrotate(b2,-rp.Orientation);
x3=imrotate(x2,-rp.Orientation);

[b4,bb4]=crop(b3);
x4=crop(x3,bb4);


x=x4;
b=b4;


    




