function [x,b,pt3d]=readImage(idx)

%IDIR='C:\danijels\Matlab\cogLearn\data\objects\';
%IDIR='';
%IDIR='C:\danijels\Matlab\data\cogLearn\shapes\';

global Dirs Data
%IDIR=[Dirs.data 'shapes/'];
x0=imread([Dirs.data Data.imgName num2str(idx,['%0' num2str(Data.numDigit) 'd']) Data.imgFormat]);
b0=imread([Dirs.data Data.mskName num2str(idx,['%0' num2str(Data.numDigit) 'd']) Data.imgFormat]);
try
    pt3d = load([Dirs.data Data.ptsName num2str(idx,['%0' num2str(Data.numDigit) 'd']) Data.ptsFormat]) ;
    if size(pt3d,2)>3
        pt3d=pt3d(:,4:6);
        b0=(b0==1);
    end
catch
    pt3d = [] ; warning('3d data not available?') ;
end

%b0=imread([Dirs.data 'msk' num2str(idx,'%03d') '.jpg']);



b1=bin(b0);
%b1=(b0==120);
[b2,bb2]=crop(b1);
x2=crop(x0,bb2);

x3=double(x2);
b=b2;

x4=x3.*repmat(b,[1 1 3]);
x=uint8(x4);

x=x2;
