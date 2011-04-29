%function [Fall,AP,CF]=readAllImages1



%%%%%%%%%%%% LOAD DATA %%%%%%%%%%%%%%%%%%%


attNames=['Rd';'Gr';'Bl';'Yl';'Wh';'Bk';'Gy';'Mt';'Or';'Pp';'Pk';'Br';'Tn';'Sm';'Lr';'Cm';'El';'Cr';'Rn';'Ln';'Sh'];

numAV=21;
numF=6;

DIRimgs='C:\danijels\Matlab\data\cogLearn\objects\';
N1=1;
N2=400;
trImgs=N1:N2;
N=length(trImgs);

Fall=zeros(numF,N);


%load training images
for i=1:N

   dwaitbar(i/N, 'Loading training images...');

   imgfile=[DIRimgs 'can/img' num2str(trImgs(i),'%03d') ,'.jpg'];
   maskfile=[DIRimgs 'can/msk' num2str(trImgs(i),'%03d') ,'.jpg'];
   datfile=[DIRimgs 'dat' num2str(trImgs(i),'%03d') ,'.mat'];

   x=imread(imgfile);
   b=imread(maskfile);
   load(datfile,'gt');

   [fAVa,Fnames]=extFeatures(x,b);
   Fall(:,i)=fAVa;

   imshow(x);drawnow;

end

[AP,CF]=readAPs(trImgs,numAV,DIRimgs);

AV=lf2sf(AP);
AVlf=AP;

F=Fall;
C=AP;
