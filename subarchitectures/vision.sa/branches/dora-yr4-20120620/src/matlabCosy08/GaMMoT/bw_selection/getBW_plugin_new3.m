function covariance = getBW_plugin( kernel_pdf, x_new, prior_adapt, sg, n_sg ) 
% Matej Kristan (2007)
%
% A two-stage plugin rule for bandwidth selection 
%
% Fist use Silverman and assume the underlying distribution is Gaussian
% to obtain a rough estimate of the bandwidths for the incoming samples.
% Augment the distribtuion from previous time-step by the kernels using 
% the new samples. Use this distribtion to approximate the integral of 
% the squared second derivative of the target distribution. Use this 
% result to reestimate the bandwidth, and scale the estimated bandwidth
% by factor 1.3 to arrive at the final estimate.
 
global h_plugin ;

it_max = 20 ;
d = max(rows(x_new),rows(kernel_pdf.mu) ) ;

N_eff = n_sg ;
Mu2k = d ;
Rk = (1/(2*sqrt(pi)))^d ;

Const1 = ( Rk/(Mu2k^2)/N_eff )^(2/(d+4)) ;
% initialize covariance
C0 = getSilvermanBWfromGaussian( sg, n_sg ) ;

%  
Rp0 = evalR2( kernel_pdf.mu, kernel_pdf.covariances, kernel_pdf.weights ) ;   
C0 = Const1* Rp0^(-2/(d+4));
    



scl = 1.2 ;  %  TALE PARAMETER JE ZA PREPREÈEVANJE UNDERSMOOTHINGA
for i = 1 : 1
   scale = 1 ;  
   kernel_pdf_1 = augmentMixtureWithCurrentData( kernel_pdf, prior_adapt, C0*scale , x_new ) ;
%   kernel_pdf_1 = kernel_pdf ;
  
   Rp1 = evalR2( kernel_pdf_1.mu, kernel_pdf_1.covariances, kernel_pdf_1.weights ) ;   
   C1 = Const1* Rp1^(-2/(d+4)) *scl^2 ;
   C0 = C1 /scl^2 ;
end
scl = 1;%1.3^2; 1.3^2 ; %1.229^2 ;
covariance = C1 * scl ; 

h_plugin = sqrt(covariance) ;

function F = opt_h_functionMy(h,data)
 kernel_pdf_1 = augmentMixtureWithCurrentData( data.kernel_pdf, data.prior_adapt, h^2, data.x_new ) ;
 Rp2 = evalR2( kernel_pdf_1.mu, kernel_pdf_1.covariances, kernel_pdf_1.weights ) ;
 F = h - sqrt(data.Const1* Rp2^(-2/(data.d+4))) ;



% --------------------------------------------------------- % 
function kernel_pdf = augmentMixtureWithCurrentData( kernel_pdf, prior_adapt, covariance, x_new ) 
kernel_pdf.mu = [ kernel_pdf.mu, x_new ] ;
kernel_pdf.weights = [ kernel_pdf.weights*kernel_pdf.components*(1-prior_adapt), 1.0*1.0*prior_adapt ] ; 
kernel_pdf.weights  = kernel_pdf.weights  / sum(kernel_pdf.weights ) ;
kernel_pdf.covariances = [ kernel_pdf.covariances; covariance ] ;
kernel_pdf.components = kernel_pdf.components + 1.0 ;
kernel_pdf.weights = kernel_pdf.weights / sum(kernel_pdf.weights) ;
    