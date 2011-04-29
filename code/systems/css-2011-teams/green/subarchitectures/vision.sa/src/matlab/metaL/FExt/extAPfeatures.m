function [F,rgb3d]=extAPfeatures(X,B,FV,pts3d)
%[F,Fnames]=extAPfeatures(X,B)
%Extract appearance properties features.
%Extract simple apperarnce and shape features.
%Returns feature names if no input argument is given.
%X: RGB images of the arbitrary size
%B: corresponding segmentation masks of the same size
%F: vector of feature values
%Fnames: feature acronyms

%features' variantS

if nargin < 3
   FV=5;%4;
end;   

if nargin==1
   FV=X;
end   

if nargin < 4
    pts3d = [] ;
end

switch FV
    case 1, Fnames=['Hu';'Sa';'In';'Ar';'Pr';'Co'];
    case 2, Fnames=['Hu';'Sa';'In';'Ar';'Cp';'Ec'];
    case 3, Fnames=['Hu';'Sa';'In';'S1';'S2';'S3';'S4'];
    case 4, Fnames=['Hu';'Sa';'In';'S1';'S2';'S3';'S4';'S5'];
    case 5, Fnames=['Hu';'Sa';'In';'S1';'S2';'S3'];
end


if nargin<=1
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
      
% %       idx = randperm(size(V,1)) ;
% %       num_sel = min([100, length(idx) ]) ; 
% %       data = V(idx(1:num_sel),:) ; 
% % %       N_max = min([size(V,1),100]) ;
% % %       idx = round(linspace(1,size(x,1),N_max)) ;
% % %       data = V(idx,:) ; 
% %       [clustCent,num_in_cluster] = getModesInData( data ) ;
% %       if ~isempty(num_in_cluster)
% %           medHSV = clustCent(:,1) ;
% %       else
% %          medHSV = median(V)';  
% %       end
% medHSV = median(V)';  
      if length(medHSV) == 3        
        [hu,sa,in]=deal(medHSV(1),medHSV(2),medHSV(3));
      else
        hu = -1 ;
        sa = -1 ;
        in = -1 ;
      end
          
      if in==1 
%          in
         in=in-abs(rand*1e-6); 
%          in
      end; %saturated intensity values...

      if isnan(hu)
        hu = -1 ;
        sa = -1 ;
        in = -1 ;
      end
      
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

      if ~isempty(pts3d)
          if size(pts3d,2) > 3
              rgb3d = pts3d(:,4:6) ;
              pts3d = pts3d(:,1:3) ;
          else
              rgb3d = [] ;
          end
          shp3d = extractShpFts( pts3d ) ;
      else
          shp3d = [ -666, -666, -666 ] ;
      end
      
      
      %COMPOUND DATA
      switch FV
         case 1, F(:,ii)=[hu;sa;in;ar;pr;co];
         case 2, F(:,ii)=[hu;sa;in;ar;df;ec];
         case 3, F(:,ii)=[hu;sa;in;mg';sum(mg)];
         case 4, F(:,ii)=[hu;sa;in;mg';sum(mg);ec];
         case 5, F(:,ii)=[hu; sa; in; shp3d(1); shp3d(2); shp3d(3)];    
      end

   end;
   
   %showROI(X,B,F);
   
end

if nargout < 2
    rgb3d = [] ;
end
