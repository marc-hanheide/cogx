% getallshapefeatures
%
% Calculate a selection of shape features of a binary object.
%
% Possible syntax:
% [Features, FeatureNames] = getallshapefeatures(Image)
%
% Input: Image = 2D Binary Image containing an object.
%
% Output: Features = A selection of shape features.
%         FeatureNames = The names of those features.
%

function [Features, FeatureNames] = getallshapefeatures(Image)

    if ndims(Image) ~= 2 || max(max(Image)) ~= 1
        Image = getbiggestobject(Image);
    end
    
    STATS = regionprops(double(Image), 'all');
    
    Features(1) = STATS.Area;
    FeatureNames{1} = 'Area';
    Features(2) = STATS.ConvexArea;
    FeatureNames{2} = 'ConvexArea';
    Features(3) = STATS.Eccentricity;
    FeatureNames{3} = 'Eccentricity';
    Features(4) = STATS.EquivDiameter;
    FeatureNames{4} = 'EquivDiameter';
    Features(5) = STATS.EulerNumber;
    FeatureNames{5} = 'EulerNumber';
    Features(6) = STATS.Extent;
    FeatureNames{6} = 'Extent';
    Features(7) = STATS.FilledArea;
    FeatureNames{7} = 'FilledArea';
    Features(8) = STATS.MajorAxisLength;
    FeatureNames{8} = 'MajorAxisLength';
    Features(9) = STATS.MinorAxisLength;
    FeatureNames{9} = 'MinorAxisLength';
    Features(10) = STATS.Perimeter;
    FeatureNames{10} = 'Perimeter';