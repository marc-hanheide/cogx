function covariance = getBW_plugin_tresh( kernel_pdf, x_new, prior_adapt, sg, n_sg ) 
% Matej Kristan (2007)
%
% A plug in rule for bandwidth selection uder a Gaussian
% assumption
%

global h_plugin ;

it_max = 20 ;
d = max(rows(x_new),rows(kernel_pdf.mu) ) ;

% % construct pdf for gaussian
% pdf_gauss.mu = [0] ;
% pdf_gauss.covariances = [sg] ;
% pdf_gauss.weights = [1] ;
% % calculate scale of the input distribution
% [scale shift]= getScaleOfDistribution( kernel_pdf ) ;
% rescale the distribution
% kernel_pdf = rescaleDistribution( kernel_pdf, scale, shift, 'forward' ) ;
% pdf_gauss = rescaleDistribution( pdf_gauss, scale, shift, 'forward' ) ;
% sg = pdf_gauss.covariances ;

N_eff = n_sg ;
Mu2k = d ;
Rk = (1/(2*sqrt(pi)))^d ;

Const1 = ( Rk/(Mu2k^2)/N_eff )^(2/(d+4)) ;
% initialize covariance
C0 = getSilvermanBWfromGaussian( sg, n_sg )  ;

% covariance = C0*4 ;
% if ( n_sg > 2 )
%     Rp2 = evalR2( kernel_pdf.mu, kernel_pdf.covariances, kernel_pdf.weights ) ;
%     C1 = Const1* Rp2^(-2/(d+4)) ;
%     covariance = C1 ; 
% end


% Rp2 = evalR2( pdf_gauss.mu, pdf_gauss.covariances, pdf_gauss.weights ) ;
% covariance = Const1*( Rp2^(-2/5) )*0.5^2 ;

% iterate until convergence
%tol = 1e-13 ;

% data.kernel_pdf = kernel_pdf ;
% data.prior_adapt = prior_adapt ;
% data.h0 = sqrt(C0) ;
% data.x_new = x_new ;
% data.d = d ;
% data.Const1 = Const1 ;

 kernel_pdf_x.mu = sum(kernel_pdf.mu .* kernel_pdf.weights,2) ;
 kernel_pdf_x.covariances = C0 ;sg ;C0 ;
 kernel_pdf_x.weights = 1 ;sg ;C0 ;
    
 
%  Rp2 = evalR2( kernel_pdf.mu, kernel_pdf.covariances, kernel_pdf.weights ) ;
%  C0 = Const1* Rp2^(-2/(d+4)) ;
%  
CC = [];
for i = 1 : 1
    c = 1;%0.9^2 ; % (3*(35)^(-1/5))^2 % *N_eff.^(2/10)
    
   scale = 1; 0.5^2 ;
   %  kernel_pdf_1 = kernel_pdf ;
%          kernel_pdf_1 = augmentMixtureWithCurrentData( kernel_pdf, prior_adapt, C0*scale*N_eff.^(2/10) , x_new ) ;%
           kernel_pdf_1 = augmentMixtureWithCurrentData( kernel_pdf, prior_adapt, C0*scale , x_new ) ;
%         kernel_pdf_1.mu = [kernel_pdf.mu, kernel_pdf_x.mu] ;
%         kernel_pdf_1.covariances = [kernel_pdf.covariances; kernel_pdf_x.covariances] ;
%         kernel_pdf_1.weights = [kernel_pdf.weights*0.5, 0.5] ;

%     kernel_pdf_1.mu = [kernel_pdf_1.mu, kernel_pdf_x.mu] ;
%     kernel_pdf_1.covariances = [kernel_pdf_1.covariances; kernel_pdf_x.covariances] ;
%     kernel_pdf_1.weights = [kernel_pdf_1.weights*0.5, 0.5] ;

   % kernel_pdf_1.weights = kernel_pdf_1.weights / sum(kernel_pdf_1.weights ) ;
    
     %kernel_pdf_1 = augmentMixtureWithCurrentData( kernel_pdf, prior_adapt, C0 , x_new ) ;
%      kernel_pdf_1 = kernel_pdf ; 
%     kernel_pdf_1.covariances = kernel_pdf_1.covariances * N_eff^(2/10) ;
    
%     w = kernel_pdf_1.weights(length(kernel_pdf_1.weights)) ;
%     w =w*1.4 ; 
%     kernel_pdf_1.weights(length(kernel_pdf_1.weights)) = w ; 
%     kernel_pdf_1.weights = kernel_pdf_1.weights /sum(kernel_pdf_1.weights) ;
              Rp2 = evalR2( kernel_pdf_1.mu, kernel_pdf_1.covariances, kernel_pdf_1.weights ) ;
%              Rp2 = evalR2( kernel_pdf_x.mu, kernel_pdf_x.covariances, kernel_pdf_x.weights ) ;
 
    C1 = Const1* Rp2^(-2/(d+4)) ;
    
    
   CC = [CC, [C0;C1]] ;
   s = ((C0 - C1).^2)/d^2 ;
%    if ( s < tol ) break ; end

   if i == 1 
       C2 = C1 ;
   end


   s = ((C0 - C1).^2)/d^2 ;
 
   C0 = C1 ;
 
    C0 = C1 ;
    
    
end
covariance = C1  ;
 

C1 = covariance ; C2 = covariance ;
h_plugin =( sqrt([ C2; C1 ]) );
% options = optimset('Display','off','TolFun',1e-14,'TolX',1e-14,'LargeScale','on') ;
% [h,resnorm,residual,exitflag,output] = lsqnonlin(@opt_h_functionMy,data.h0,[0],[],options,data) ;
% covariance = h.^2 ;

% pdf_gauss.covariances = [covariance] ;
% pdf_gauss = rescaleDistribution( pdf_gauss, scale, shift, 'backward' ) ;
% covariance = pdf_gauss.covariances ;

%figure(2); clf; plot(CC(1,:),CC(2,:));


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
    