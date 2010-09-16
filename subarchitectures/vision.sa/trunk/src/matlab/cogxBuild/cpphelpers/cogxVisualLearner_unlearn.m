%% UNLEARN %% 
%**orig**: this->eval(navw.str().c_str());
%**orig**: this->eval("[mC,mDA,mFS]=KDBFunlearn(features, avw, mC, mDA, mFS)");
%**orig**: this->eval("mFS.AVmeans");
%**orig**: this->eval("mFS.AVvars");
%**orig**: this->eval("LRvisUpdate");
function cogxVisualLearner_unlearn(avw, X, B, pts3d)
   global mC
   
   global Params
   B = double(B);
   B = (B==1);
   X = uint8(X);
   f = extAPfeatures(X,B,Params.FV,pts3d);

   mC = MKDBFunlearn(f, avw(:,1), mC);
   LRvisUpdate;
   asvSave;
   disp(['MATLAB: cogxVisualLearner_unlearn DONE']);
end
