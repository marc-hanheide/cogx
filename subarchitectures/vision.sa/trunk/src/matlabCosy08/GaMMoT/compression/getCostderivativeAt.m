function dX = getCostderivativeAt( f_ref, f_curr )

len_current = cols(f_curr.mu) ;
d = rows(f_curr.mu) ;
alpha = zeros(len_current, len_current) ;
gamma = zeros(len_current, len_current) ;
eta = zeros(len_current, len_current) ;
alpha_diag = zeros(1,len_current) ;
eta_diag = zeros(1,len_current) ;
for i = 1 : len_current
    Cov1 = reshape(f_curr.covariances(i, :),d,d) ;
    mu1 = f_curr.mu(:,i) ;
    alpha_diag(i) = integOfTwoGaussProd( mu1, Cov1, mu1, Cov1 ) ;
    eta_diag(i) = -0.5/ det(Cov1) ;  
    for j = i+1 : len_current
        Cov2 = reshape(f_curr.covariances(j, :), d, d) ;
        mu2 = f_curr.mu(:,j) ;
        alpha(i,j) = integOfTwoGaussProd( mu1, Cov1, mu2, Cov2 ) ;
        Cov0 =( Cov1 + Cov2 ) ;
        gamma(i,j) = (mu1 - mu2) / Cov0 ;
        eta(i,j) = (gamma(i,j).^2-1/Cov0) ;
    end
end
alpha = (alpha + alpha') ;
gamma = gamma - gamma' ;
eta = eta + eta' + diag(eta_diag) ;

len_ref = cols(f_ref.mu) ; 
beta = zeros(len_current, len_ref) ;
delta = zeros(len_current, len_ref) ;
ceta = zeros(len_current, len_ref) ;
for i = 1 : len_current
    Cov1 = reshape(f_curr.covariances(i, :),d,d) ;
    mu1 = f_curr.mu(:,i) ;
    for j = 1 : len_ref
        Cov2 = reshape(f_ref.covariances(j, :), d, d) ;
        mu2 = f_ref.mu(:,j) ;
        beta(i,j) = integOfTwoGaussProd( mu1, Cov1, mu2, Cov2 ) ;
        Cov0 =( Cov1 + Cov2 )  ;
        delta(i,j) = (mu1 - mu2) / Cov0 ;
        ceta(i,j) = (delta(i,j)^2-1/ Cov0)  ;
    end
end
w2 = repmat(f_ref.weights,length(f_curr.weights),1) ;
w1 = repmat(f_curr.weights,length(f_curr.weights),1) ;

dW = wCostderivative( alpha, alpha_diag, beta, f_ref.weights, f_curr.weights, w1, w2 ) ;
dMu = muCostderivative( alpha, beta, gamma, delta, f_curr.weights, w1, w2 ) ;
dCov = covCostderivative( alpha+diag(alpha_diag), beta, eta, ceta, f_curr.weights, w1, w2  ) ;

dX = [ dMu; dCov; dW ] ;

function dW = wCostderivative( alpha, alpha_diag, beta, weights_ref, weights_current, w1, w2 )
dW = 2*sum(w1.*alpha,2) + 2*(weights_current.*alpha_diag)' - 2*sum(w2.*beta,2) ;

function dMu = muCostderivative( alpha, beta, gamma, delta, weights_current, w1, w2 )
dMu = 2*(-weights_current'.*sum(w1.*alpha.*gamma,2) + weights_current'.*sum(w2.*beta.*delta,2) ) ;

function dCov = covCostderivative( alpha, beta, eta, ceta, weights_current, w1, w2  )
dCov = weights_current'.*sum(w1.*alpha.*eta,2) - weights_current'.*sum(w2.*beta.*ceta,2)  ;

