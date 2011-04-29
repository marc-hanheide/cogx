function kernel_pdf = updateKDE_plugin(kernel_pdf, kernel_x0, ...
                                          x_new, sg, n_sg,...
                                          prior_adapt, force_dims  ) 
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


if ( isempty(prior_adapt) ) prior_adapt = 0.5 ; end

% take care of dimensionality
if ( isempty(force_dims) | force_dims == Inf | force_dims > length(x_new) )
   force_dims = length(x_new) ; 
end
kernel_x0 = kernel_x0(1:force_dims) ;
x_new = x_new(1:force_dims) ;
sg = sg(1:force_dims) ;

% estimate the current covariance of the data
covariance = getBW_plugin( kernel_pdf, sg, n_sg ) ;
 
% verify if we're in a initialization phase 
[kernel_pdf, rtrn ] = verifyInitialization( kernel_pdf, kernel_x0, x_new, covariance ) ;
if rtrn == 1 return ; end

% augment existing components with the new component
kernel_pdf = augmentMixtureWithCurrentData( kernel_pdf, prior_adapt, covariance, x_new ) ;
 
% --------------------------------------------------------- % 
function kernel_pdf = augmentMixtureWithCurrentData( kernel_pdf, prior_adapt, covariance, x_new ) ;
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
