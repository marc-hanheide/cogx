function R = evalR2( mu, covariances, weights )
%
% Matej Kristan (2007)
%
% calculates R(p'') = int( diff2(p)^2 dx )
%
%
%
%

warning off;

d = rows(mu) ;
n_data = length(weights) ;

UT = getUTPairs(n_data) ;
num_p = cols(UT) ;
% get Z values and determinants H for offdiagonal elements
Z_ut = zeros(1,num_p) ;
H_ut = zeros(1,num_p) ;
W_ut = zeros(1,num_p) ;
for i = 1 : num_p
    j = UT(1,i) ; 
    k = UT(2,i) ; 
    mu1 = mu(:,j) ;
    mu2 = mu(:,k) ;
    C1 = reshape( covariances(j,:), d, d) ;
    C2 = reshape( covariances(k,:), d, d) ;
    C = C1 + C2 ;
    m = mu1 - mu2 ;
    Z_ut(i) = sqrt(m' * inv(C) * m) ;
    H_ut(i) = sqrt(det(C)) ;
    W_ut(i) = weights(j)*weights(k) ;
end

% get Z values and determinants for diagonal elements
Z_d = zeros(1,n_data) ;
H_d = zeros(1,n_data) ;
W_d = zeros(1,n_data) ;
for i = 1 : n_data
    C = reshape( covariances(i,:), d, d)*2 ;
    m = 0 ;
    Z_d(i) = sqrt(m' * inv(C) * m) ;
    H_d(i) = sqrt(det(C)) ;
    W_d(i) = weights(i)*weights(i) ;
end

% evaluate integrals
I_ut = 0 ;
if ( ~isempty(Z_ut) )
    I_ut = evalDifIntK_r4( Z_ut, H_ut ) ;   
end
I_d  = evalDifIntK_r4( Z_d, H_d ) ;

R = 2*sum(I_ut.*W_ut) + sum(I_d.*W_d) ;

warning on;


% ------------------------------------------------ %
function d = evalDifIntK_r4( Z, H )
% int( diff(K1,2), diff(K2,2) )
% 1/2*(3-6*u^2+u^4)*2^(1/2)/pi^(1/2)*exp(-1/2*u^2)
d = 7186705221432913/18014398509481984*(3-6*Z.^2+Z.^4).*exp(-1/2*Z.^2)./H.^5 ;


% ------------------------------------------------ %
function UT = getUTPairs(N)
% first generate upper triangle pairs
UT = [] ;
for i = 2 : N
 
    I = [i:N] ;
    UT = [UT, [I; 0*I+(i-1)]] ;
end