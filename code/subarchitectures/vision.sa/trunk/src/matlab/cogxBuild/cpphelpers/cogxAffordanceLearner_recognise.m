function Affordance = cogxAffordanceLearner_recognise(Features)
%
% Note: This assumes the existence of a global BiModalLearner object named
% AffordanceClassifier
% 

    FeaturesResults = AffordanceClassifier.classify(Features);
    Affordance = FeaturesResults.Results.GroundTruthClassification{1};

end
