function bel = getBeliefOfdata( pdf, data )

p = evaluateDistributionAt( pdf.mu, pdf.weights, pdf.covariances, data ) ;
bel = p/pdf.max.val ;
bel = (bel<1).*bel + (bel>=1) ;
%bel = min([bel,1]) ;

