% process3Dshapefeatures
%

function [Features, FeatureNames] = processallfeatures(FileSpec, varargin)

    Foo = regexp(FileSpec, '[0-9]');
    
    [TwoDFeatures TwoDFeatureNames] = process2Dshapefeatures([FileSpec(1:Foo(end)) 'm.png']);
    
    % Knock out the Euler Number feature
    TwoDFeatures = TwoDFeatures(:, [1:4 6:end]);
    
    try
        [ThreeDFeatures ThreeDFeatureNames] = process3Dshapefeatures([FileSpec(1:Foo(end)) 'points'],...
                                                                     'z_scaling_factor', 2.5);
    catch Error1
        try
            [ThreeDFeatures ThreeDFeatureNames] = process3Dshapefeatures([FileSpec(1:Foo(end)) 'patch'],...
                                                                         'z_scaling_factor', 2.5);
        catch Error2
            rethrow(Error2);
        end
    end
    
    Features = [ThreeDFeatures TwoDFeatures];
    
    FeatureNames = [ThreeDFeatureNames TwoDFeatureNames];