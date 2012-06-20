function F = cosyFeatureExtractor_extract(X, B)
   global Params
   X=uint8(X);
   F = extAPfeatures(X,B,Params.FV);
   showROI(X,B,F);
end
