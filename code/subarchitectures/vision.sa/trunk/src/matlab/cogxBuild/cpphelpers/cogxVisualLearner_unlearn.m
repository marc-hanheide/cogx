%% UNLEARN %% 
%**orig**: this->eval(navw.str().c_str());
%**orig**: this->eval("[mAV,mDA,mFS]=KDBFunlearn(features, avw, mAV, mDA, mFS)");
%**orig**: this->eval("mFS.AVmeans");
%**orig**: this->eval("mFS.AVvars");
%**orig**: this->eval("LRvisUpdate");
function cogxVisualLearner_unlearn(features, avw)
   global mAV
   global currMode

   mAV = MKDBFunlearn(features, avw(:,1), mAV);
   LRvisUpdate;
   asvSave;
   disp(['MATLAB: cogxVisualLearner_unlearn DONE']);
end
