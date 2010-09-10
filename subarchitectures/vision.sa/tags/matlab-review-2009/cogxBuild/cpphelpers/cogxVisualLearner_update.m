%% UPDATE %%
%**orig**: this->eval(avw.str().c_str());
%**orig**: this->eval("[mAV,mDA,mFS]=KDBFupdate(features, avw, mAV, mDA, mFS)");
%**orig**: this->eval("mFS.AVmeans");
%**orig**: this->eval("mFS.AVvars");
%**orig**: this->eval("LRvisUpdate");
function cogxVisualLearner_update(avw,X,B,pts3d)
   global mAV
   global avAcronyms
   
   global Params
   B = double(B);
   B = (B==1);
   X = uint8(X);
   f = extAPfeatures(X,B,Params.FV,pts3d)


  % mAV = MKDBFupdate(f, avw, mAV);
%   mAV = MKDBFupdate(f, [avw(:,1) ones(size(avw,1),1)], mAV);
   mAV = MKDBFupdate(f, avw(:,1), mAV);

disp(['Updated: ' idx2name(avw(:,1)',avAcronyms)]);
   LRvisUpdate;
   asvSave;
   disp(['MATLAB: cogxVisualLearner_update DONE']);
end
