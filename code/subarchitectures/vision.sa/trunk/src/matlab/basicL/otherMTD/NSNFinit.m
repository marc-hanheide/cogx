function [mAV,mDA,mFS]=NSNFinit
%[mAV,mDA,mFS]=MVBFinit
%MVBF initialisation.
%mAV: model of AVs
%mDA: model of detected attributes
%mFS: feature statistics


mFS=struct('Fmean',[],'Fvar',[],'Fn',[],'Fmeans',[],'Fcvars',[],'Fns',[]);

%AV (attribute values)
mAV=struct('name', [], 'Fvar', [], 'mean', [], 'EV', [], 'vars', [], 'conf', []);

%DA (detected attributes)
mDA=struct('Fb',[],'AV',[]);



