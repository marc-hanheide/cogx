%% UNLEARN %% 
%**orig**: this->eval(navw.str().c_str());
%**orig**: this->eval("[mAV,mDA,mFS]=KDBFunlearn(features, avw, mAV, mDA, mFS)");
%**orig**: this->eval("mFS.AVmeans");
%**orig**: this->eval("mFS.AVvars");
%**orig**: this->eval("LRvisUpdate");
function cogxVisualLearner_unlearn(avw, X, B, pts3d)
   global mAV
   
   global Params
   B = double(B);
   B = (B==1);
   X = uint8(X);
   f = extAPfeatures(X,B,Params.FV,pts3d);

   mAV = MKDBFunlearn(f, avw(:,1), mAV);
   LRvisUpdate;
   asvSave;
   disp(['MATLAB: cogxVisualLearner_unlearn DONE']);
end
