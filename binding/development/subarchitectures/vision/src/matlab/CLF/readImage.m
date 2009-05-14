function x=readImage(idx)

%IDIR='C:\danijels\Matlab\cogLearn\data\objects\';
IDIR='';
%IDIR='C:\danijels\Matlab\data\cogLearn\shapes\';
IDIR='/home/user/matlab/data/shapes/';

x0=imread([IDIR 'img' num2str(idx,'%03d') '.jpg']);
b0=imread([IDIR 'msk' num2str(idx,'%03d') '.jpg']);


b1=bin(b0);
[b2,bb2]=crop(b1);
x2=crop(x0,bb2);

x3=double(x2);
b=b2;

x4=x3.*repmat(b,[1 1 3]);
x=uint8(x4);

