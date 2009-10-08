function [ansYes, ansPy, answ] = cogxVisualLearner_recognise(X, B)
   % X - image
   % B - segmentation mask
   % Copied from cosyFeatureExtractor_limitvalue
   B = double(B)
   B(B > 1.0) = 1.0

   % Copied from cosyFeatureExtractor_extract
   global Params
   X = uint8(X);
   F = extAPfeatures(X,B,Params.FV);
   showROI(X,B,F);

   % Copied from cosyRecogniser_recogise
   readConstants;
   global mAV mDA mFS
   global currMode
   global ANSyes ANSpy
   global avAcronyms

   answ = KDBFrec(f, mAV, mFS);
   if currMode.qnt2qlD==0
      ansQl = qnt2ql(answ, currMode.THRs);
   else
      ansQl = qnt2qlD(answ, currMode.THRs, currMode.CTT);
   end

   ansYes = lf2sfa(ansQl, ANSyes);
   ansPy = lf2sfa(ansQl, ANSpy);
   disp(['Recognised: ',idx2name(ansYes,avAcronyms)]);
   showRec(ansYes,ansPy,answ,f);
end
