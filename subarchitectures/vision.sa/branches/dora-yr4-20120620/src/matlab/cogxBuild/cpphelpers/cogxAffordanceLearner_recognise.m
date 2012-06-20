function Affordance = cogxAffordanceLearner_recognise(Features)
%
% Note: This assumes the existence of a global BiModalLearner object named
% Classifier
% 
    global Classifier 
    if isempty(Features) || length(Features) ~= 11
        Affordance = 'Undefined';
        return
    end

    FeaturesResults = Classifier.gtclassify('data', Features');
    res = FeaturesResults.Results.GroundTruthClassification;
    Affordance = res{1};

    displayTD(Affordance);
    
    disp('cogxAffordanceLearner_recognise done.');
end
