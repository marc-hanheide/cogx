function F = opt_mod_function(h,data)

lambda=data.constant2*h^(5/7);

% [D4]=UnivariateDensityDerivative(data.N,data.N,data.X,data.X,lambda,4);
% phi4=sum(D4)/(data.N-1);
pdf = readjustKernels( data.pdf, lambda^2 ) ;
phi4 = intFunctional( data.pdf, 0, 4 ) ;
 
F=h-data.constant1*phi4^(-1/5);