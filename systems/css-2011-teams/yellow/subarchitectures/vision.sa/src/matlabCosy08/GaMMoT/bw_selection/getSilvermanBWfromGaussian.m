function covariance = getSilvermanBWfromGaussian( sg, n_sg ) 
% Matej Kristan (2007)
%
% A simple Silverman rule for bandwidth selection uder a Gaussian
% assumption
%

scale_out_cov =1; 0.4^2 ;
% estimate new bandwidth
bandwidth = bandwidthFromGauss( sqrt(sg), n_sg ) ;
% transform to covariance
tmp_cov = diag(bandwidth.^2) ;
covariance = reshape(tmp_cov, 1,cols(tmp_cov)*rows(tmp_cov)) ;
covariance = covariance * scale_out_cov ; 
%covariance = max(covariance,sg*10^(-2) )  ; 

% --------------------------------------------------------- %                                                  
function bandwidth = bandwidthFromGauss( sigma, N )
% calculates bandwidth estimate of kernel for the new data
% sigma ... current standard deviation
% N ... current number of data in the distribution
bandwidth = sigma*(4/(3*N))^(1/5) ;