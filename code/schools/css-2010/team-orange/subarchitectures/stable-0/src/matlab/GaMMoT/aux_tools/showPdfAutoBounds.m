function showPdfAutoBounds( N, centers, covariances, weights, color )

s = 6 ;
d_min = centers-covariances'*s ;
d_max = centers+covariances'*s ;
bounds = [min(d_min),max(d_max)] ;
showPdfNonNorm( bounds, N,centers, covariances, weights, color ) ;