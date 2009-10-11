%% UPDATE %%
%**orig**: this->eval(avw.str().c_str());
%**orig**: this->eval("[mAV,mDA,mFS]=KDBFupdate(features, avw, mAV, mDA, mFS)");
%**orig**: this->eval("mFS.AVmeans");
%**orig**: this->eval("mFS.AVvars");
%**orig**: this->eval("LRvisUpdate");
function cosyRecogniser_update(features, avw)
   global mAV mDA mFS
   global avAcronyms

   [mAV,mDA,mFS]=KDBFupdate(features, avw, mAV, mDA, mFS);
   disp(['Updated: ' idx2name(avw(:,1)',avAcronyms)]);
   LRvisUpdate;
   asvSave;
end
