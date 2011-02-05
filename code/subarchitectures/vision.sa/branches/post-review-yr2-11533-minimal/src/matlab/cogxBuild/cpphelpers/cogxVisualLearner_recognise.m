function [rCpcx,gain] = cogxVisualLearner_recognise(X, B, pts3d)
   % X - image - byte_array
   % B - segmentation mask - byte_array
   global Figs

   B = double(B); % TODO: is it really necessary to convert the mask to double?
   B = (B==1);
   if sum(B(:)) == 0
      rCqnt = [];
      rCpcx = [];
      gain = [];
   else
      disp(['MATLAB: cogxVisualLearner_recognise']);
      % Copied from cosyFeatureExtractor_extract
      global Params
      
      %resize mask to match image size
      sizex=size(X);
      if ~isequal(sizex(1:2),size(B))
          B=imresize(B,sizex(1:2), 'nearest');
      end

      X = uint8(X);
      try
          f = extAPfeatures(X,B,Params.FV,pts3d);
      catch
          f = [] ;
          rCqnt = [];
          rCpcx = [];
          gain = [];
          return ;
      end
      showROI(X,B,f,pts3d);

      % figure('color','k');
      % image(X);

      % Copied from cosyRecogniser_recogise
      readConstants;
      global mC
      global currMode
      % global ANSyes ANSpy
      global Coma       
      
%       rCqnt = MKDBFrec(f, mC);
%       if currMode.qnt2qlD==0
%         ansQl = qnt2ql(rCqnt, Params.THRs);
%       else
%         ansQl = qnt2qlD(rCqnt, Params.THRs, Coma.SCC);
%       end
%       ansYes = lf2sfa(ansQl, ANSyes);
%       ansPy = lf2sfa(ansQl, ANSpy);
%       disp(['Recognised: ',idx2name(rCqnt,Coma.Cnames)]);
%       showRec(ansYes,ansPy,rCqnt,f);

      avu=ODKDErec(f,mC); %recognize AVs considering current models
      answ=avu;
      rCpcx=cc2c(answ,'trim');
      ansQl=qnt2ql(rCpcx,Params.THRs);
      ansYes = lf2sfa(ansQl, ANSyes);
      ansPy = lf2sfa(ansQl, ANSpy);
      disp(['Recognised: ',idx2name(ansYes,Coma.Cnames)]);
      
      g=ODKDEgain(f,mC);
      gain=cc2c(g);
      
      division_at = size(avu{1},1)-1 ;
      showRec(ansYes,ansPy,rCpcx,f,division_at);
      displayTR(ansYes,ansPy,avu,g);
      displayG(Figs.LRguiR.main,'GR');                 
      
      disp(['MATLAB: cogxVisualLearner_recognise DONE']);
      
      %LRvisUpdate;
   end
    
end
