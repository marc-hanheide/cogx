function [X,AV]=genShapes(numVar,varSiz,varCol,varNoise)
%[X,AV]=genShapes(numVar,varSiz,varCol,varNoise)
%Generate shapes.
%Generate basic shapes of different colors, sizes and shapes.
%numVar: number of generated shapes for each combination of visual
%attributes.
%varSiz,varCol,varNoise: variances of the ize, color, and added gaussian
%noise.
%X: generated images in (IWH,IWH,3,N) matrix
%AV: ground truth AVs in a long form



TF=2;

IWH=100;
numAtt=3;
numShp=4;
numCol=4;
numSiz=2;

shapes{1}=imread('sqr1.bmp');
shapes{2}=imread('crc1.bmp');
shapes{3}=imread('trg1.bmp');
shapes{4}=imread('rct1.bmp');

colors{1}=[1 0 0];
colors{2}=[0 1 0];
colors{3}=[0 0 1];
colors{4}=[1 1 0];

sizes{1}=.5;
sizes{2}=1;

N=numShp*numCol*numSiz*numVar;
X=zeros(IWH,IWH,3,N);
A=zeros(numAtt,N);
i=0;
for shp=1:numShp;
   for col=1:numCol
      for siz=1:numSiz
         for var=1:numVar
            i=i+1;
            Xi1=~shapes{shp};
            %Xi2(:,:,1)=Xi1*colors{col}(1)*tnormrnd(.5,varCol,TF);
            Xi2(:,:,1)=Xi1*(colors{col}(1)+tnormrnd(.1,varCol,TF))*tnormrnd(.5,varCol,TF);
            Xi2(:,:,2)=Xi1*colors{col}(2)*tnormrnd(.5,varCol,TF);
            Xi2(:,:,3)=Xi1*colors{col}(3)*tnormrnd(.5,varCol,TF);
            Xi3=imresize(Xi2,sizes{siz}*tnormrnd(1,varSiz,TF));
            Xi4=zeros(IWH*2,IWH*2,3);
            Xi4(IWH-floor(size(Xi3,1)/2):IWH+ceil(size(Xi3,1)/2)-1,IWH-floor(size(Xi3,2)/2):IWH+ceil(size(Xi3,1)/2)-1,:)=Xi3;
            Xi5=Xi4(IWH/2:3*IWH/2-1,IWH/2:3*IWH/2-1,:);
            Xi6=Xi5+abs(normrnd(0,varNoise,[IWH,IWH,3]));
            X(:,:,:,i)=Xi6;
            A(:,i)=[col;siz;shp];
         end
      end
   end
end;   

A(2,:)=A(2,:)+max(A(1,:));%AV: Rd,Gr,Bl,Sm,Lr,Sq,Cr,Tr,Rc
A(3,:)=A(3,:)+max(A(2,:));
numAV=max(A(:));

X=abs(X);
AV=sf2lf(A,numAV);

