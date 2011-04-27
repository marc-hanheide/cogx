%Read all images, extract features and read gt values


Cnames=['TL';'TR';'CT';'FT';'NT';'FF';'OL';'IM';'OR';'NR';'FA'];
Fnames=['x ';'y ';'dx';'dy';'d '];


numC=11;
numF=5;

%imgDir='C:\danijels\Matlab\cogLearn\annotate\objects3\objects\';
imgDir='C:\danijels\Matlab\data\cogLearn\relations\';
N1=1;
N2=300;

imgIdx=N1:N2;
N=length(imgIdx);

Fall=zeros(numF,N);

[R,P]=readRs(imgIdx,numC,imgDir);

%load images
for i=1:N

   dwaitbar(i/N, 'Loading and processing images...');

   [f1]=extRfeatures(P(:,2*i-1),P(:,2*i));
   [f2]=extRfeatures(P(:,2*i),P(:,2*i-1));
   Fall(:,2*i-1:2*i)=[f1 f2];

%   imshow(x);drawnow;

end


F=Fall;
C=R;

imgIdx=1:size(R,2);

F=F+rand(size(F))*1e-4;

