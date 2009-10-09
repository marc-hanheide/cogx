%% UNLEARN %% 
%**orig**: this->eval(navw.str().c_str());
%**orig**: this->eval("[mAV,mDA,mFS]=KDBFunlearn(features, avw, mAV, mDA, mFS)");
%**orig**: this->eval("mFS.AVmeans");
%**orig**: this->eval("mFS.AVvars");
%**orig**: this->eval("LRvisUpdate");
function cosyRecogniser_unlearn(features, avw)
   global mAV mDA mFS
   global currMode

   [mAV,mDA,mFS]=KDBFunlearn(features, avw(:,1), mAV, mDA, mFS);
   LRvisUpdate;
   asvSave;
end
