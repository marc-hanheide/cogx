function [Features] = cogxAffordanceLearner_getFeatures(Image, Mask, Points3D, Patches)
   
    [TwoDFeatures TwoDFeatureNames] = process2Dshapefeatures(Mask);
    
    % Knock out the Euler Number feature
    TwoDFeatures = TwoDFeatures(:, [1:4 6:end]);
    
    [ThreeDFeatures ThreeDFeatureNames] = process3Dshapefeatures(Patches);
    
    Features = [ThreeDFeatures TwoDFeatures];
end
