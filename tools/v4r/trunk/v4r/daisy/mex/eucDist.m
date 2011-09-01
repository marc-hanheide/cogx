function [distMap,minDist,minC,minR] = sqEucDist(FullDesc, DescToComp)
% SQEUCDIST calculates a squared Euclidean distance map
% 
% [DISTMAP,MINDIST,MINC,MINR] = SQEUCDIST(FULLDESC,DESCTOCOMP) calculates 
% the squared Euclidean distance between the dense descriptor map FULLDESC 
% and the DESCTOCOMP.
% 
% Parameters:
%   FULLDESC      Full Descriptor of the dimension L x W x H where
%                   L   is the length of the descriptor,
%                   W   is the number of columns and
%                   H   is the number of rows of the descriptor map
%
%   DESCTOCOMP    Descriptor that should be compared to FULLDESC
%                 The dimension of DESCTOCOMP must be 1 x L or L x 1
%
% Returns:
%   DISTMAP       A W*H x 3 matrix which holds the squared Euclidean distance
%                 between each descriptor in an array with the entry
%                 DISTANCE W H
%                 The array can be easly sorted using
%                 distMap = sqEucDist(FullDesc, DescToComp)
%                 [sv si]  = sort(distMap(:,1));
%                 distMap = distMap(si,:);
%
%   MINDIST       The minimum distance found in the distmap
%   MINC          The column of the cell containing the minimum distance
%   MINR          The row of the cell containing the minimum distance
%
% Author: pb (Philipp Blauensteiner)
% Version: 0.2

% Version History
% 0.2   added MINDIST, MINC, MINR

% Version History
% Author: mb (Markus Bader)
% 0.3   change DISTMAP  =     a [W*H x 3] matrix


    if (nargin ~= 2)
        error('Please mind the usage');
    end
    if (size(FullDesc,1)~=max(size(DescToComp)))
        error('Please mind the usage');
    end
    [distMap, minDist, minC, minR] = mex_eucDist(FullDesc, DescToComp);

end