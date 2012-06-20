function vals = getNonlinThresholdFun( pdf, N )

%
% Matej Kristan (2007)
%
% Generates an ordered sample set of distribution values
%

X = sampleMixtureOfGaussians( pdf.mu, pdf.weights, pdf.covariances, N ) ;
vals = evaluateDistributionAt( pdf.mu, pdf.weights, pdf.covariances, X ) ;

vals = sort(vals) ;


