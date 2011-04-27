%% UNLEARN %% 
function cogxVisualLearner_unlearn(avw, X, B, pts3d)
   global mC
   global Params Coma
   
   %resize mask to match image size
   sizex=size(X);
   if ~isequal(sizex(1:2),size(B))
       B=imresize(B,sizex(1:2), 'nearest');
   end
   
   B = double(B);
   B = (B==1);
   X = uint8(X);
   f = extAPfeatures(X,B,Params.FV,pts3d);

%   mC = MKDBFunlearn(f, avw(:,1), mC);
   c=avw2snf(avw(:,1),Coma.SCC);
   mC=ODKDEupdate(f,c,mC);
      
   LRvisUpdate;
   asvSave;
   disp(['MATLAB: cogxVisualLearner_unlearn DONE']);
end
