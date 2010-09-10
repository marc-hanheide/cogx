%AP10
%Read all images, extract features and read gt values

%function [Fall,AP,CF]=readAllImages1


Cnames=['Rd';'Gr';'Bl';'Yl';'Sm';'Lr';'Sq';'Tr';'Cr';'Rc'];

numC=10;
Fnames=extAPfeatures(1);
numF=length(Fnames);

imgDir='C:\danijels\Matlab\cmLearn\Sim\data\shapes\';
N1=1;
N2=300;

imgIdx=N1:N2;
N=length(imgIdx);

Fall=zeros(numF,N);

%load images
for i=1:N

   dwaitbar(i/N, 'Loading and processing images...');

   imgfile=[imgDir 'img' num2str(imgIdx(i),'%03d') ,'.jpg'];
   maskfile=[imgDir 'msk' num2str(imgIdx(i),'%03d') ,'.jpg'];

   x=imread(imgfile);
   b=imread(maskfile);

   f=extAPfeatures(x,b,1);
   Fall(:,i)=f;

   imshow(x);drawnow;

end

[AP,CF]=readAPs(imgIdx,numC,imgDir);

F=Fall;
C=AP;

