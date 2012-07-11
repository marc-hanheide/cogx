function [x,b,pt3d]=getImage

global Data;
global Test;


if ~Test.genColImg
    [x,b,pt3d]=readImage(Data.currImg);
else
    x=Test.x*255;
    b=Test.b;
    pt3d=Test.pt3d;
end
