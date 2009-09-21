function [kernel_pdf, C_cov] = updateKDE(kernel_pdf, kernel_x0, ...
                                          x_new, sg, n_sg,...
                                          prior_adapt, force_dims,...
                                          C_sensor, mu_sg ) 
% kernel_pdf ... current parameters of distribution
%                 kernel_pdf.mu 
%                 kernel_pdf.weights
%                 kernel_pdf.covariances
%                 kernel_pdf.components
%
% kernel_x0 ... initial point used for initialization of the distribution
% x_new ... current data
% sg ... component-wise variances for bandwidth estimation
% n_sg ... number of components used to calculate "sg"
% prior_adapt ... a priori adaptation by the new data 
%                 if empty, i.e. [], then default "prior_adapt = 0.5" 
%                 is used.
% nMaxComponents ... maximum number of components in kernel estimate before
%                    merging is carried out
% force_dims ... force number of dimensions (default = Inf)

C_cov = [] ;
covMinValue = 10e-6 ;  10e-8 ;

if ( isempty(prior_adapt) ) prior_adapt = 0.5 ; end

% take care of dimensionality
if ( isempty(force_dims) | force_dims == Inf | force_dims > length(x_new) )
   force_dims = length(x_new) ; 
end
if isempty(C_sensor)
    C_sensor = 0 ;
end

if ( ~isempty(kernel_x0) )
    kernel_x0 = kernel_x0(1:force_dims) ;
end
x_new = x_new(1:force_dims) ;
sg = sg(1:force_dims) ;
 
       [covariance, pars ,Csil, Wsil ] = getBW_plugin( kernel_pdf, x_new, prior_adapt, sg, n_sg, mu_sg ) ;
%        [covariance, pars ,Csil, Wsil ] = getBW_plugin( kernel_pdf, x_new, prior_adapt, sg, n_sg, mu_sg, 0 ) ;
%        disp(sprintf('Bw prev: %1.3g, Bw new: %1.3g', covariance, covariance1 )) ;
       
       covariance = covariance + C_sensor ;
       kernel_pdf.pars = pars ;
%        [length(kernel_pdf.weights),Wsil],pause(0.1)
%       if kernel_pdf.H0 == -1 kernel_pdf.H0 = covariance_p ; end
%       if kernel_pdf.components > 1
%           scala = covariance_p/kernel_pdf.H0 ;
%           len = length(kernel_pdf.weights) ;
%           kernel_pdf.covariances(1:len) = kernel_pdf.covariances(1:len) *scala ;
%           kernel_pdf.H0 = covariance_p ;
%       end
      
 
% end
% covariance = covariance_sil ;

% covariance =  covariance*(1-Wsil) + Wsil*Csil ;

%msg = sprintf('Selected bandwidth^2: %g, Silverman bandwidth^2: %g', (covariance_p), covariance_sil) ; disp(msg) ;
% covariance = 0.0633^2 
% verify if we're in a initialization phase 

if ( covariance < covMinValue )
    covariance = covMinValue ;
%    figure(1); title('Covariance constrained!!!!!!') ;
end

rtrn = 0 ;
if ( ~isempty(kernel_x0) )
    [kernel_pdf, rtrn ] = verifyInitialization( kernel_pdf, kernel_x0, x_new, covariance ) ;
end
if rtrn == 1 
    if nargout == 2  C_cov = covariance ; end
    return ; 
end
 
% augment existing components with the new component
kernel_pdf = augmentMixtureWithCurrentData( kernel_pdf, prior_adapt, covariance, x_new ) ;
 
% kernel_pdf.covariances = kernel_pdf.covariances*(1-Wsil) + Wsil*Csil ;
if nargout == 2 C_cov = kernel_pdf.covariances(length(kernel_pdf.weights)) ; end

% --------------------------------------------------------- % 
function kernel_pdf = augmentMixtureWithCurrentData( kernel_pdf, prior_adapt, covariance, x_new ) ;

% correction = covariance / kernel_pdf.H0
% kernel_pdf.H0 = covariance

kernel_pdf.mu = [ kernel_pdf.mu, x_new ] ;
kernel_pdf.weights = [ kernel_pdf.weights*kernel_pdf.components*(1-prior_adapt), 1.0*1.0*prior_adapt ] ; 
kernel_pdf.weights  = kernel_pdf.weights  / sum(kernel_pdf.weights ) ;
kernel_pdf.covariances = [ kernel_pdf.covariances; covariance ] ;
kernel_pdf.components = kernel_pdf.components + 1.0 ;
kernel_pdf.weights = kernel_pdf.weights / sum(kernel_pdf.weights) ;

% --------------------------------------------------------- %
function [kernel_pdf, rtrn ] = verifyInitialization( kernel_pdf, kernel_x0, x_new, covariance )
rtrn = 0 ;
% verify if pdf needs to be initialized
if ( isempty(kernel_pdf.mu) )
   kernel_pdf.mu = [kernel_x0, x_new] ;
   kernel_pdf.weights = [0.5, 0.5] ;
   kernel_pdf.covariances = [covariance; covariance] ;
   kernel_pdf.components = 2 ; 
   rtrn = 1 ;
end
