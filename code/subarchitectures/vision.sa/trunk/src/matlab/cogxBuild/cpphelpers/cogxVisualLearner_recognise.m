function [rCqnt] = cogxVisualLearner_recognise(X, B, pts3d)
   % X - image - byte_array
   % B - segmentation mask - byte_array
   % Copied from cosyFeatureExtractor_limitvalue
  
   %figure(666) ; 
   %subplot(1,2,1) ; imagesc(uint8(X)) ;
   %subplot(1,2,2) ; imagesc(uint8(B)) ; colormap gray ;
   
   %msg = sprintf('Color intens range: %1.2g , % 1.2g', min(X(:)), max(X(:))) ; disp(msg) ;
   %msg = sprintf('Mask intens range: %1.2g , % 1.2g', min(B(:)), max(B(:))) ; disp(msg) ;
   %figure(667) ; 
      
   % pts3d(1:10,:)
   B = double(B); % TODO: is it really necessary to convert the mask to double?
   % B = (B==120);
   B = (B==1);
   if sum(B(:)) == 0
      rCqnt = [];
   else
      disp(['MATLAB: cogxVisualLearner_recognise']);
      % Copied from cosyFeatureExtractor_extract
      global Params
      X = uint8(X);
      f = extAPfeatures(X,B,Params.FV,pts3d);
      showROI(X,B,f,pts3d);

      % figure('color','k');
      % image(X);

      % Copied from cosyRecogniser_recogise
      readConstants;
      global mC
      global currMode
      % global ANSyes ANSpy
      global Coma 

      rCqnt = MKDBFrec(f, mC);
      if currMode.qnt2qlD==0
        ansQl = qnt2ql(rCqnt, Params.THRs);
      else
        ansQl = qnt2qlD(rCqnt, Params.THRs, Coma.SCC);
      end

      ansYes = lf2sfa(ansQl, ANSyes);
      ansPy = lf2sfa(ansQl, ANSpy);

      disp(['Recognised: ',idx2name(rCqnt,Coma.Cnames)]);
      showRec(ansYes,ansPy,rCqnt,f);
      disp(['MATLAB: cogxVisualLearner_recognise DONE']);
      
      LRvisUpdate;
   end
    
end
