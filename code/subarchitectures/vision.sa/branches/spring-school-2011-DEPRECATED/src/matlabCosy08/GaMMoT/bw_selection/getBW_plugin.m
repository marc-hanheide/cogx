function [covariance, pars, Csil, Wsil ]= getBW_plugin( kernel_pdf, x_new, prior_adapt, sg, n_sg, mu_sg ) 
% Matej Kristan (2007)
%
% An iterative plugin rule for bandwidth selection 
%

global h_plugin N_eff ;
it_max = 20 ;
N_eff = n_sg ;
if nargin < 6
    mu_sg = [] ; %x_new ;
end
 
% calculate the effective sample size
wt = prior_adapt/( (1-prior_adapt)*kernel_pdf.components + prior_adapt ) ;
wt1 = kernel_pdf.pars.wt1 ;
Wt1 = kernel_pdf.pars.Wt1 ;
Wt = (1-wt1)^2 * Wt1+ wt1^2 ;
N_eff = ((1-wt)^2 * Wt + wt^2)^(-1) ;


% if kernel_pdf.components == 100 ||kernel_pdf.components == 150
%     fg = 67 ;
% end
 
N_eff = kernel_pdf.components + 1 ;
 
% figure(3); title(sprintf('effective sample size: N_{eff}=%f',N_eff)) ;

pars.Wt1 = Wt ;  
pars.wt1 = wt ; 
 
d = max(rows(x_new),rows(kernel_pdf.mu) ) ;

Mu2k = d ;
Rk = (1/(2*sqrt(pi)))^d ;
Const1 = ( Rk/(Mu2k^2)/N_eff )^(2/(d+4)) ;


% % initialize covariance
% C0 = getSilvermanBWfromGaussian( sg, n_sg ) ;
% for i = 1 : 1
%    scale = 1 ;  
%    kernel_pdf_1 = augmentMixtureWithCurrentData( kernel_pdf, prior_adapt, C0*scale , x_new ) ;
% %   kernel_pdf_1 = kernel_pdf ;
%   
%    Rp1 = evalR2( kernel_pdf_1.mu, kernel_pdf_1.covariances, kernel_pdf_1.weights ) ;   
%    C1 = Const1* Rp1^(-2/(d+4)) ;
% end
% scl = 1.3^2 ; %1.229^2 ;
% covariance = C1 * scl ; 

silverScale = 1.0 ;
% inflate pdf using silverman regularization of derivative
% C0 = silverScale^2*(sqrt(sg)*(4/(3*n_sg))^(1/5))^2 ; % silverman rule-of-thumb 
% w_mix = 1 - getMixingSilvermanWeight( n_sg, 0.1 ) ; % 0.5  0.8 ;
% Csil = sqrt(sg) / n_sg ; %C0 ;
% Wsil = w_mix ;
% kernel_pdf.covariances = kernel_pdf.covariances*(1-Wsil) + Wsil*Csil ;
% kernel_pdf.covariances = [kernel_pdf.covariances; Csil] ;
% kernel_pdf.mu = [kernel_pdf.mu, mu_sg] ;
% kernel_pdf.weights = [kernel_pdf.weights*(1-Wsil),Wsil] ;

Rp0 = evalR2( kernel_pdf.mu, kernel_pdf.covariances, kernel_pdf.weights ) ;   
C0 = Const1* Rp0^(-2/(d+4));
    
silverScale = 1 ; %0.5 ;

N = kernel_pdf.components ;

scl = 1.0 ;
for i = 1 : 6
   scale = 1 ;  
   kernel_pdf_1 = augmentMixtureWithCurrentData( kernel_pdf, prior_adapt, C0*scale , x_new ) ;
%   kernel_pdf_1 = kernel_pdf ;
  
   Rp1 = evalR2( kernel_pdf_1.mu, kernel_pdf_1.covariances, kernel_pdf_1.weights ) ;   
   C1 = Const1* Rp1^(-2/(d+4)) *scl^2 ;
   C0 = C1 /scl^2 ;
end
scl = 1; % 1.3^2;% 1.3^2 ; %1.229^2 ;
%scl = (1./(N./chi2inv(0.90,N-1))) ; 
scl = 1 ; %+ 1*sqrt(2)./sqrt(N-1) ;
covariance = C1 * scl ; 
% 
C0 = silverScale^2*(sqrt(sg)*(4/(3*n_sg))^(1/5))^2 ; % silverman rule-of-thumb 
% lambda = 4 ; w_mix = exp(-N/lambda) ;
% silverScale = 0.5 ;
% w_mix = 1 -  getMixingSilvermanWeight( n_sg, 0.1 ) ;
w_mix = 1 - getMixingSilvermanWeight( n_sg, 0.9 ) ; % 0.5  0.8 ;

Csil = C0 ; Wsil = w_mix ;

% covariance = C0*w_mix + (1.0 - w_mix)*covariance ;

% sample-point estimation of bandwidth
% covariance = samplePointEstimate(kernel_pdf, prior_adapt, covariance, x_new) ;

% bet = (N - 1)/2 ;
% alph = (N-1)/( 2* C1) ;
% covariance = gaminv(0.5,bet+1,alph^(-1)) ;

h_plugin = sqrt(covariance) ;

function res = getMixingSilvermanWeight( N, percent )
v = 1 ;
bet = (N - 1)/2 ;
alph = (N-1)/( 2* v) ;
A = percent*sqrt( (N-1)/2 ) ;
res = gamcdf(1 + A,bet+1, alph.^(-1)) - gamcdf(1 - A , bet+1, alph.^(-1)) ;


function covariance = samplePointEstimate(kernel_pdf, prior_adapt, covariance, x_new)

kernel_pdf_1 = augmentMixtureWithCurrentData( kernel_pdf, prior_adapt, covariance , x_new ) ;


pdf = evaluateDistributionAt( kernel_pdf_1.mu, kernel_pdf_1.weights, kernel_pdf_1.covariances, kernel_pdf_1.mu ) ;
lam = exp((1/length(kernel_pdf_1.mu))*sum(log(pdf))) ;

covariance = covariance*( lam/pdf(length(pdf)) ) ;



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
    