function [mC,mDA,mFS]=NSNFinit
%[mC,mDA,mFS]=MVBFinit
%MVBF initialisation.
%mC: model of AVs
%mDA: model of detected attributes
%mFS: feature statistics


mFS=struct('Fmean',[],'Fvar',[],'Fn',[],'Fmeans',[],'Fcvars',[],'Fns',[]);

%AV (attribute values)
mC=struct('name', [], 'Fvar', [], 'mean', [], 'EV', [], 'vars', [], 'conf', []);

%DA (detected attributes)
mDA=struct('Fb',[],'AV',[]);



