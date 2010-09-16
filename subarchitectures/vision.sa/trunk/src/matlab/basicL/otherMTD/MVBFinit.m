function [mC,mDA,mFS]=MVBFinit
%[mC,mDA,mFS]=MVBFinit
%MVBF initialisation.
%mC: model of AVs
%mDA: model of detected attributes
%mFS: feature statistics


mFS=struct('Fmean',[],'Fvar',[],'Fn',[],'Fmeans',[],'Fvars',[],'Fns',[]);

%AV (attribute values)
mC=struct('name', [], 'mean', [], 'var', [], 'Fb', [], 'conf', []);

%DA (detected attributes)
mDA=struct('Fb',[],'AV',[]);



