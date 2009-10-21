function [mAV,mDA,mFS]=MVBFinit
%[mAV,mDA,mFS]=MVBFinit
%MVBF initialisation.
%mAV: model of AVs
%mDA: model of detected attributes
%mFS: feature statistics


mFS=struct('Fmean',[],'Fvar',[],'Fn',[],'Fmeans',[],'Fvars',[],'Fns',[]);

%AV (attribute values)
mAV=struct('name', [], 'mean', [], 'var', [], 'Fb', [], 'conf', []);

%DA (detected attributes)
mDA=struct('Fb',[],'AV',[]);



