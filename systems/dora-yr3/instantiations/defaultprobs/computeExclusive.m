function independentGenerationLambda = computeExclusive(generalProbability, ...
    otherObjectGenerationLambdas, otherObjectHasLambdas)

%generalProbability: Probability that this location has
%at least one object, directly or indirectly

%independentGenerationLambda: Rate at which this location
%generates the reference object independently

%otherObjectGenereationLambdas: Rate at which each of
%possible location objects under this location are generated

%otherObjectHasLambdas: Rate at which each of possible
%location objects under this location in turn generate the
%ref object
nums = 0:20;

numOtherObjectTypes = size(otherObjectGenerationLambdas(:),1);

% The probability that none of the "other" object classes
% contains any of the reference object
probOtherObjectsHaveNone = 1;

for otherObject = 1:numOtherObjectTypes
    % Multiply together probabilities of each class of "other"
    % objects containing no ref object
    probOfCounts = poisspdf(nums, otherObjectGenerationLambdas(otherObject));
    
    % Probability that each location object is empty
    probOtherObjectHasNone = poisspdf(0,otherObjectHasLambdas(otherObject));
    
    % Vector of probabilities, given a certain count of
    % location objects, that none of the objects have a 
    % ref object
    probOfNothingPerCount = probOtherObjectHasNone.^nums;
    
    % Vector of probabilities that there will be a certain
    % amount of location objects, all empty
    probOfNothingUncond = probOfCounts.*probOfNothingPerCount;
    
    % Probability that there is no object of class 
    % otherObj containing any reference object
    probOfNothing = sum(probOfNothingUncond);
    
    probOtherObjectsHaveNone = ...
        probOtherObjectsHaveNone * probOfNothing;
end

% Probability that location generates no reference objects
probGeneratesNone = (1-generalProbability)/probOtherObjectsHaveNone;

% Poisson distribution lambda that gives the above as its 0
independentGenerationLambda = -log(probGeneratesNone);