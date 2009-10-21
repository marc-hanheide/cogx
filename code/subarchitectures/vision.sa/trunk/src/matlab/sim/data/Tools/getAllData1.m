%Read all images, extract features and read gt values

%function [Fall,AP,CF]=readAllImages1


%Cnames=['Rd';'Gr';'Bl';'Yl';'Wh';'Bk';'Gy';'Mt';'Or';'Pp';'Pk';'Br';'Tn';'Sm';'Lr';'Cm';'El';'Cr';'Rn';'Ln';'Sh'];
%Cnames=['Rd';'Gr';'Bl';'Yl';'Sm';'Lr';'Sq';'Tr';'Cr';'Rc'];
Cnames=['Rd';'Gr';'Bl';'Yl';'Sq';'Tr';'Cr';'El'];

numC=8;
Fnames=extAPfeatures;
numF=length(Fnames);

imgDir='C:\danijels\Matlab\cogLearn\data\objects2\';
N1=1;
N2=296;

imgIdx=N1:N2;
N=length(imgIdx);

Fall=zeros(numF,N);

%load images
for i=1:N

   dwaitbar(i/N, 'Loading and processing images...');

   imgfile=[imgDir 'img' num2str(imgIdx(i),'%03d') ,'.png'];
   maskfile=[imgDir 'msk' num2str(imgIdx(i),'%03d') ,'.png'];

   x=imread(imgfile);
   b=imread(maskfile);

   f=extAPfeatures(x,b,4);
   Fall(:,i)=f;

   imshow(x);drawnow;

end

%[AP,CF]=readAPs(imgIdx,numC,imgDir);

F=Fall;
%C=AP;

