function F=extAPfeatures(X,B,FV)
%[F,Fnames]=extAPfeatures(X,B)
%Extract appearance properties features.
%Extract simple apperarnce and shape features.
%Returns feature names if no input argument is given.
%X: RGB images of the arbitrary size
%B: corresponding segmentation masks of the same size
%F: vector of feature values
%Fnames: feature acronyms

%features' variantS

if nargin<3
   FV=4;
end;   

switch FV
   case 1, Fnames=['Hu';'Sa';'In';'Ar';'Pr';'Co'];
   case 2, Fnames=['Hu';'Sa';'In';'Ar';'Cp';'Ec'];
   case 3, Fnames=['Hu';'Sa';'In';'S1';'S2';'S3';'S4'];
   case 4, Fnames=['Hu';'Sa';'In';'S1';'S2';'S3';'S4';'S5'];
end


if nargin==0
   F=Fnames;
else

%figure;imshow(X);
%figure;imshow(X/255);
%figure;imshow(B);


   [IH,IW,foo,N]=size(X);
   numF=size(Fnames,1);
   F=zeros(numF,N);

   for ii=1:N
      x=X(:,:,:,ii);
      b1=B(:,:,ii);
      b=b1/max(double(b1(:)));

      %GET APPEARANCE FEATURES
      HSV=rgb2hsv(x);
      %HSV(HSV(:,:,1)>.8)=0; %red color...
      idxs=find(HSV(:,:,1)>.8);HSV(idxs)=1-HSV(idxs); %red color...

      %consider foreground only
      idxs=find(b==1);
      V0=reshape(HSV,IH*IW,1,3);
      V=V0(idxs,:);

      %get median
      medHSV=median(V)';
      [hu,sa,in]=deal(medHSV(1),medHSV(2),medHSV(3));


      %GET SHAPE FEATURES
      if FV <3
         bd=double(b);
         rp=regionprops(bd,'Area','Perimeter','MajorAxisLength','MinorAxisLength','Eccentricity');

         ar=rp.Area/1e5;%numel(b); %area
         pr=rp.Perimeter+4; %perimeter
         co=pr*pr/rp.Area; %compactness

         df=rp.MajorAxisLength/rp.MinorAxisLength;
         ec=rp.Eccentricity;
      else

         img=b1;
         img=(img>0); 
         % the size of image is:
         imgsz=size(img);

         % find the centroid of an object (centeri,centerj)
         [j,i]=meshgrid(1:imgsz(2),1:imgsz(1));
         ipixels=find(img);
         centeri=mean(i(ipixels));
         centerj=mean(j(ipixels));

         % object-centered coordinates ci,cj:
         ci=i(ipixels)-centeri;  % these are centered coords
         cj=j(ipixels)-centerj;

         % convert them to radial coords
         r=sqrt(ci.^2+cj.^2);
         phi=atan2(ci,cj);

         % make angular histogram, vote by radius or so
         Nsteps=32; % angular steps within 2*pi
         step=2*pi/Nsteps;
         f=0:step:2*pi-sqrt(eps);

         % radius normalization - for scale invariance:
         r=r/sqrt(length(ipixels));
         phi=phi+pi; % this makes phi to be in (0,2*pi)
         % for each phi, compute to which bin it falls:
         histbin=floor(phi/step)+1; histbin(histbin>Nsteps)=Nsteps;
         % compute the histogram (voting by r)
         anghist=sparse(1,histbin,r,1,Nsteps);

         % normalize once again - now using area under histogram
         anghist=anghist/sum(anghist);

         % extract fourier magnitudes (2 to 4 cycles/period)
         mg=zeros(1,3);
         for jj=2:4
            mg(jj-1)=sqrt(sum(sin(f*jj).*anghist)^2+sum(cos(f*jj).*anghist)^2);
         end


      end

      if FV ==4
         bd=double(b);
         rp=regionprops(bd,'Eccentricity');
         ec=rp.Eccentricity;
      end


      %COMPOUND DATA
      switch FV
         case 1, F(:,ii)=[hu;sa;in;ar;pr;co];
         case 2, F(:,ii)=[hu;sa;in;ar;df;ec];
         case 3, F(:,ii)=[hu;sa;in;mg';sum(mg)];
         case 4, F(:,ii)=[hu;sa;in;mg';sum(mg);ec];
      end

   end;
   
   %showROI(X,B,F);
   
end