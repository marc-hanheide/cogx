function F=extAPfeatures(X,B)
%[F,Fnames]=extAPfeatures(X,B)
%Extract appearance properties features.
%Extract simple apperarnce and shape features.
%Returns feature names if no input argument is given.
%X: RGB images of the arbitrary size
%B: corresponding segmentation masks of the same size
%F: vector of feature values
%Fnames: feature acronyms

%features' variantS
FV=1;

switch FV
   case 1, Fnames=['Hu';'Sa';'In';'Ar';'Pr';'Co'];
   case 2, Fnames=['Hu';'Sa';'In';'Ar';'Cp';'Ec'];
end

if nargin==0
   F=Fnames;
else
   [IH,IW,foo,N]=size(X);
   numF=size(Fnames,1);
   F=zeros(numF,N);

   for i=1:N
      x=X(:,:,:,i);
      b=B(:,:,i);
      b=b/max(double(b(:)));

      %GET APPEARANCE FEATURES
      HSV=rgb2hsv(x);
      HSV(HSV(:,:,1)>.8)=0; %red color...
      %idxs=find(HSV(:,:,1)>.8);HSV(idxs)=1-HSV(idxs); %red color...

      %consider foreground only
      idxs=find(b==1);
      V0=reshape(HSV,IH*IW,1,3);
      V=V0(idxs,:);

      %get median
      medHSV=median(V)';
      [hu,sa,in]=deal(medHSV(1),medHSV(2),medHSV(3));


      %GET SHAPE FEATURES
      bd=double(b);
      rp=regionprops(bd,'Area','Perimeter','MajorAxisLength','MinorAxisLength','Eccentricity');

      ar=rp.Area/1e5;%numel(b); %area
      pr=rp.Perimeter+4; %perimeter
      co=pr*pr/rp.Area; %compactness

      df=rp.MajorAxisLength/rp.MinorAxisLength;
      ec=rp.Eccentricity;


      %COMPOUND DATA
      switch FV
         case 1, F(:,i)=[hu;sa;in;ar;pr;co];
         case 2, F(:,i)=[hu;sa;in;ar;df;ec];
      end

   end;
end