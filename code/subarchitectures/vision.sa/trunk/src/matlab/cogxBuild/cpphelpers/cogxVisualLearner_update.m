%% UPDATE %%
function cogxVisualLearner_update(avw,X,B,pts3d)
   global mC Coma Params

   B = double(B);
   B = (B==1);
   X = uint8(X);
   f = extAPfeatures(X,B,Params.FV,pts3d);


   % mC = MKDBFupdate(f, avw, mC);
   % mC = MKDBFupdate(f, [avw(:,1) ones(size(avw,1),1)], mC);
   % mC = MKDBFupdate(f, avw(:,1), mC);
   c=avw2snf(avw(:,1),Coma.SCC);
   f
   mC=ODKDEupdate(f,c,mC);

   disp(['Updated: ' idx2name(avw(:,1)',Coma.Cnames)]);
   LRvisUpdate;
   asvSave;
   disp(['MATLAB: cogxVisualLearner_update DONE']);
end
