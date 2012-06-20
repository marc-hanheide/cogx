function outMeasure = calculateClAic (X, w, mu, Sigma)

global globalUseGammaPdf ;

len_data = size(X,1) ;

% evaluate the mixture-wise likelihoods
pyn_th = eval_comp_wise_likelihood( X ,mu, Sigma ) ;

% prepare weight matrix 
if ( size(w,1) > size(w,2) ) w = w' ; end
W = repmat(w,len_data,1) ;

% calculate weighted mixtures
wk_pyn_th = W.*pyn_th ;
sum_wk_pyn_th = sum(wk_pyn_th,2) ;
SUM_wk_pyn_th = repmat( sum_wk_pyn_th, 1, size(wk_pyn_th,2) ) ;

py_zk = wk_pyn_th ./ SUM_wk_pyn_th ;

% classify data to mixtures
Z = ( (py_zk - repmat(max(py_zk')', 1, size(py_zk,2)))==0 ) ;
Z = Z ./ repmat(sum(Z,2),1, size(Z,2)) ;

% calculate the assignments log-likelihood
p_temp = sum(Z.*py_zk,2) ;
B = sum( log( p_temp ) ) ;

% calculate data log-likelihood
A = sum(log( sum(wk_pyn_th,2) )) ;

Ck = sum(size(w)) + sum(size(mu)) + size(Sigma,2) ;

ClAic = -A -B + Ck ;

Bic = -A + (Ck/2)*log(len_data) ;
outMeasure = Bic ; % CLAic ;Bic

function pyn_th = eval_comp_wise_likelihood( X ,mu, Sigma )

pyn_th = [] ;
for i = 1 : rows(mu)
    pyn_th = [pyn_th, normpdf(X', mu(i,:), [] , Sigma(i,:))'] ;
end



% global globalUseGammaPdf ;
% if ( globalUseGammaPdf ~= 1 )
%     pyn_th = eval_lik_logNormal_mix(X,mu,Sigma) ; 
% else
%     pyn_th = eval_lik_Gamma(X,mu,Sigma) ;
% end



return

global globalUseGammaPdf ;


if ( globalUseGammaPdf ~= 1 )
    X(:,1) = exp(X(:,1)) ;
end

error(nargchk(4, 5, nargin));
if (nargin < 5)
  QUIET = 0;
end
[N, p, DIAG_COV] = mix_chk(w, mu, Sigma);
[T, Nc] = size(X);
if (Nc ~= p)
  error('Observation vectors have an incorrect dimension.');
end

% Compute density values
 mu_l = mu(:,1) ;
 mu_v = mu(:,2) ;
 sigma_v = [] ;
 sigma_l = [] ;
 for i_s = 1 : size(mu,1)
    sigma_l = [ sigma_l; Sigma(1 + (i_s-1)*2,1) ] ;
    sigma_v = [ sigma_v; Sigma(2 + (i_s-1)*2,2) ] ;
 end
 
if ( globalUseGammaPdf ~= 1 ) 
    sigma_l = sqrt( sigma_l ) ;
end
sigma_v = sqrt( sigma_v ) ;

gamma_v = [] ;
gamma_l = [] ;
for i = 1 : size(mu,1)
    temp_lik = lognpdf(X(:,1), mu_l(i), sigma_l(i)) ;
    gamma_l = [gamma_l, temp_lik ] ;
    gamma_v = [gamma_v, normpdf(X(:,2), mu_v(i), sigma_v(i)) ] ;
end
gamma = gamma_l.*gamma_v ;


 
 
%gamma = gauseval(X, mu, Sigma, QUIET);



% Determine the maximum a posteriori mixture components
mix_prob = gamma .* (ones(T,1) * w);

zk = zeros(size(mix_prob)) ;
for i = 1 : size(zk,1) 
    [mx_tmp,mx_i_tmp]=max(mix_prob(i,:)) ;
    zk(i,mx_i_tmp) = 1 ;
end

if ( size(gamma,2) > 1 )
    [mx,mx_i]=max(gamma') ;
    sum_mixture = sum(mix_prob')' ;
    wpk = sum((mix_prob.*zk)')' ;
else
    mx = gamma ;
    mx_i = ones(size(gamma)) ;
    sum_mixture = mix_prob ;
    wpk = mix_prob.*zk ;
end
    

pk =  wpk ./ sum_mixture ;

B = sum( log(pk) ) ;
A = sum( log(sum_mixture) ) ;
C = size(w,2) ;

ClAic = -A -B + C ;
