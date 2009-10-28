function [mC]=MKDBFinit
%[mC,mCG,mFS]=KDBFinit
%KDBF initialisation.
%mC: model of Cs
%mCG: model of detected concept groups
%mFS: feature statistics


%C (attribute values)
mC=struct('name', [], 'kde', [], 'Fb', [], 'conf', [], 'x_init', [], 'Nall', 0);




