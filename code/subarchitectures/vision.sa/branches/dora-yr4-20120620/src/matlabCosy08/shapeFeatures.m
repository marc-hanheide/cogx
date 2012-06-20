function [ar,pr,cm]=shapeFeatures(b)
%[ar,pr,cm]=shapeFeatures(b)
%Calculate shape features.
%b: binary segmentation mask of size (IWH,IWH)
%ar,pr,cm: feature values (scalars - area, perimeter, and compactness of
%the foreground region.


bd=double(b);

rp=regionprops(bd,'Area','Perimeter','MajorAxisLength','MinorAxisLength','Eccentricity');

ar=rp.Area/100000;%numel(b); %area
pr=rp.Perimeter+4; %perimeter
cm=pr*pr/rp.Area; %compactness

%rescale to app. interval [0,1] (for nicer visualisation)
%cm=(cm-10)/20;%+rand()*1e-3; 
%pr=(rp.Perimeter-50)/200;

%cm=rand;

pr=rp.MajorAxisLength/rp.MinorAxisLength;
cm=rp.Eccentricity;
 %imshow(b*255);


%beep;

return
%%%%% works better with real images
d=diameter(b);
cm=d*d/rp.Area;
pr=(d-30)/50;

