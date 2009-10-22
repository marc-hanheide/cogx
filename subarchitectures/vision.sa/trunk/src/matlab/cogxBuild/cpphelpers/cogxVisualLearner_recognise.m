function [rCqnt] = cogxVisualLearner_recognise(X, B)
   % X - image - byte_array
   % B - segmentation mask - byte_array
   % Copied from cosyFeatureExtractor_limitvalue
   B = double(B); % TODO: is it really necessary to convert the mask to double?
   B(B > 1.0) = 1.0;

   disp(['MATLAB: cogxVisualLearner_recognise']);
   % Copied from cosyFeatureExtractor_extract
   global Params
   X = uint8(X);
   f = extAPfeatures(X,B,Params.FV);
   % showROI(X,B,f);

   % figure('color','k');
   % image(X);

   % Copied from cosyRecogniser_recogise
   readConstants;
   global mC mAV
   global currMode
   % global ANSyes ANSpy
   global avAcronyms

   % Ali je answ = rCqnt ?
   rCqnt = MKDBFrec(f, mAV);
   %if currMode.qnt2qlD==0
   %   ansQl = qnt2ql(answ, currMode.THRs);
   %else
   %   ansQl = qnt2qlD(answ, currMode.THRs, currMode.CTT);
   %end

   %ansYes = lf2sfa(ansQl, ANSyes);
   %ansPy = lf2sfa(ansQl, ANSpy);
   disp(['Recognised: ',idx2name(rCqnt,avAcronyms)]);
   % ? showRec(rCqnt,f);
   disp(['MATLAB: cogxVisualLearner_recognise DONE']);
end
