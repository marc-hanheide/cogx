%% INTROSPECT %%
function gain = cogxVisualLearner_introspect

global mC

g=ODKDEintrospect(mC);
gain=cc2c(g);

displayTL(mC);
%LRvisUpdate;
%asvSave;
disp(['MATLAB: cogxVisualLearner_introspect DONE']);
end