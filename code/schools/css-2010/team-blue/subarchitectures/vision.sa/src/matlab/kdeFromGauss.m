function pdf=kdeFromGauss(mu,sg,n)

initializationMethod = 'Plugin' ; %'Silverman'; %
scaleErrorThreshold = 1/0.7 ; 1/0.7 ;1.5 ;
hellErrorGlobal = 0.1 ;% 0.11 / scaleErrorThreshold ;
nMaxComponents = 10 ;
nMaxComponentsPrior = nMaxComponents ;
initByGaussian.mu=mu;
initByGaussian.covariances=sg;
initByGaussian.num_components=n;
pdf = updateIKDE( [], [], ...
   'initialize', 1 ,...
   'nMaxComponentsPrior', nMaxComponentsPrior,...
   'scaleErrorThreshold', scaleErrorThreshold,...
   'hellErrorGlobal', hellErrorGlobal,...
   'initializationMethod', initializationMethod,...
   'initByGaussian', initByGaussian);