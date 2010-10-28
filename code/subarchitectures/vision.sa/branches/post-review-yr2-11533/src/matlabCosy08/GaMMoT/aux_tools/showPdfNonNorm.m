function showPdfNonNorm( bounds, N,centers, covariances, weights, color )

x_evals = [bounds(1):abs(diff(bounds))/N:bounds(2)] ;
y_evals = evaluateDistributionAt(centers, weights, covariances, x_evals) ;
  
plot ( x_evals, y_evals, color )