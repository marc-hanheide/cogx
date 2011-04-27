function [dL, L] = getAnalyticDerivativeAt( f_ref, f_curr )
 
len_current = cols(f_curr.mu) ;
len_ref = cols(f_ref.mu) ; 
ch_w = 'w_c' ;
ch_mu = 'mu_c' ;
ch_cov = 'C_c' ;

I = analyticIntegralOfGaussProd() ;

% get cost function
L = 0 ;
for i = 1 : len_current
    weight_i = sym(sprintf('%s%d',ch_w,i)) ;
    mu_i = sym(sprintf('%s%d',ch_mu,i)) ;
    cov_i = sym(sprintf('%s%d',ch_cov,i)) ;
    for j = 1 : len_current
        weight_j = sym(sprintf('%s%d',ch_w,j)) ;
        mu_j = sym(sprintf('%s%d',ch_mu,j)) ;
        cov_j = sym(sprintf('%s%d',ch_cov,j)) ;        
        Fi = substitute( I, mu_i, mu_j, cov_i, cov_j ) ;
        L = L + weight_i*weight_j*Fi ;
    end
end

for i = 1 : len_current
    weight_i = sym(sprintf('%s%d',ch_w,i)) ;
    mu_i = sym(sprintf('%s%d',ch_mu,i)) ;
    cov_i = sym(sprintf('%s%d',ch_cov,i)) ;
    for j = 1 : len_ref
        weight_j = f_ref.weights(j) ;
        mu_j = f_ref.mu(:,j) ;
        cov_j = f_ref.covariances(j,:) ;        
        Fi = substitute( I, mu_i, mu_j, cov_i, cov_j ) ;
        L = L - 2*weight_i*weight_j*Fi ;
    end
end

% get derivative of w1

%     dL = diff(L,sym('w_c1')) ; 
%        dL = diff(L,sym('mu_c1')) ;
   dL = diff(L,sym('C_c1'))  ;
  for i = 1 : length(f_curr.weights)
    weight_i = sym(sprintf('%s%d',ch_w,i)) ;
    mu_i = sym(sprintf('%s%d',ch_mu,i)) ;
    cov_i = sym(sprintf('%s%d',ch_cov,i)) ;
    dL = subs(dL,weight_i,f_curr.weights(i)) ;
    dL = subs(dL,mu_i,f_curr.mu(:,i)) ;
    dL = subs(dL,cov_i,f_curr.covariances(i,:)) ;
    L = subs(L,weight_i,f_curr.weights(i)) ;
    L = subs(L,mu_i,f_curr.mu(:,i)) ;
    L = subs(L,cov_i,f_curr.covariances(i,:)) ;
  end
  dL = double(dL) ;
  L = double(L) ;

% ------------------------------------------------------------------------

function I = substitute( I, mmu1, mmu2, cC1, cC2 )
syms mu1 C1 mu2 C2;

I = subs(I, mu1, mmu1) ; I = subs(I, mu2, mmu2) ;
I = subs(I, C1, cC1) ; I = subs(I, C2, cC2) ;
I = collect(I) ;


function I = analyticIntegralOfGaussProd()
syms mu1 C1 mu2 C2 Mu Cv x ;
sq2pi = sym('sqrt(2*pi)') ;
half = sym('1/2') ;
% f = (1/sqrt(2*pi*Cv))*exp(-0.5*(x-Mu)^2/Cv) ;
% F1 = subs(f,Cv,C1) ; F1 = subs(F1,Mu,mu1) ;
% F2 = subs(f,Cv,C2) ; F2 = subs(F2,Mu,mu2) ;
% I = int(F1*F2,x,-inf, inf) ;
% I = simplify(I) ;

I = 1/(sq2pi*sqrt(C1+C2))*exp(-half *(mu1-mu2)^2/(C1+C2)) ;


function w_analyticDerivative()