function [mC,mCG,mFS]=KDBFinit
%[mC,mCG,mFS]=KDBFinit
%KDBF initialisation.
%mC: model of Cs
%mCG: model of detected concept groups
%mFS: feature statistics


mFS=struct('Fmean',[],'Fvar',[],'Fn',[],'Fmeans',[],'Fvars',[],'Fns',[]);

%C (attribute values)
mC=struct('name', [], 'mu', [], 'covariances', [], 'weights', [], 'components', [], 'belFun', [], 'Fb', [], 'conf', []);

%CG (detected attributes)
mCG=struct('Fb',[],'C',[]);



