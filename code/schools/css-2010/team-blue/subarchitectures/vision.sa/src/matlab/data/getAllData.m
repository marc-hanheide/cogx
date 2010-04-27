%Read all images, extract features and read gt values

%function [Fall,AP,CF]=readAllImages1


Cnames=['Rd';'Gr';'Bl';'Yl';'Wh';'Bk';'Gy';'Mt';'Or';'Pp';'Pk';'Br';'Tn';'Sm';'Lr';'Cm';'El';'Cr';'Rn';'Ln';'Sh'];
%Cnames=['Rd';'Gr';'Bl';'Yl';'Sm';'Lr';'Sq';'Tr';'Cr';'Rc'];

numC=21;
Fnames=extAPfeatures;
numF=length(Fnames);

imgDir='C:\danijels\Matlab\data\cogLearn\objects\';
N1=1;
N2=400;

imgIdx=N1:N2;
N=length(imgIdx);

Fall=zeros(numF,N);

%load images
for i=1:N

   dwaitbar(i/N, 'Loading and processing images...');

   imgfile=[imgDir 'can\img' num2str(imgIdx(i),'%03d') ,'.jpg'];
   maskfile=[imgDir 'can\msk' num2str(imgIdx(i),'%03d') ,'.jpg'];

   x=imread(imgfile);
   b=imread(maskfile);

   f=extAPfeatures(x,b);
   Fall(:,i)=f;

   imshow(x);drawnow;

end

[AP,CF]=readAPs(imgIdx,numC,imgDir);

F=Fall;
C=AP;

