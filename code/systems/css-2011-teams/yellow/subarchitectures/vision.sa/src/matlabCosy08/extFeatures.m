%function F=extFeatures(X,B)
function [F,Fnames]=extFeatures(X,B)
%[F,Fnames]=extFeatures(X,B)
%Extract features.
%Extract simple apperarnce and shape features.
%X: RGB images of the size(IWH,IWH,3)
%B: corresponding segmentation masks of the size (IWH,IWH)
%F: vector of feature values
%Fnames: feature acronyms

Fnames=['Hu';'Sa';'In';'Ar';'Pr';'Cm'];

IWH=size(X,1);
N=size(X,4);
numF=6;
F=zeros(numF,N);

for i=1:N
   x=X(:,:,:,i);
   b=B(:,:,i);
   b=b/max(double(b(:)));

   %get appearance features
   [hu,sa,in]=appFeatures(x,b);

   %get shape features
   [ar,pr,cm]=shapeFeatures(b);

   %compound data
   F(:,i)=[hu;sa;in;ar;pr;cm];
end;

%F(6,:)=rand(1,size(F,2));