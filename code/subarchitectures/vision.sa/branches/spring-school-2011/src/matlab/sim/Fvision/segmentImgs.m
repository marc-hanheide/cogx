function B=segmentImgs(X);
%B=segmentImgs(X);
%Segment images.
%X: (a set of) RGB image(s)
%B: segmentation mask(s)

[IH,IW,foo,N]=size(X);
B=zeros(IH,IW,N);
for i=1:N
  x=rgb2gray(X(:,:,:,i));
  thr=Otsu(x);
  %thr=0;
  B(:,:,i)=x>thr;
end;  
  