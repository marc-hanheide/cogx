function equal = testForEqualityOfComponents( w0, mu0, C0, w1, mu1, C1, N, alpha )

n = ceil([w0 , w1]*N) ;
n0 = n(1) ; 
n1 = n(2) ;
n = n0 + n1 ;

% test covariances
k = 2 ;
p = rows(mu0) ;
dof = p*(p+1)*(k-1)/2 ;

if (  n0 <= 1 || n1 <= 1 || 1 < 1/(n0-1) + 1/(n1-1) - 1/(n-k)) 
    equal = 1 ; return ;
end


C = ((n0-1)*C0 + (n1 -1)*C1)/(n - k );
M = (n-k)*log(det(C)) - (  (n0-1)*log(det(C0)) + (n1-1)*log(det(C1)) ) ;
h = 1 - (2*p^2 + 3*p -1)/(6*(p+1)*(k-1))*( 1/(n0-1) + 1/(n1-1) - 1/(n-k) ) ;

% t distributes in a chi-square distribution
t = M*h ;
 
upper_bound = chi2inv(1-alpha,dof) ;
lower_bound = 0 ;% chi2inv(alpha/2,dof) ;

if ( t > chi2inv(1-alpha,dof)  )
    equal = 0 ; return ;
else
    equal = 1 ;
end

% test means
m = mu0 - mu1 ;
W = (C0*(n0-1) + C1*(n1-1))/(n0 + n1 -2) ;
 
t2 = n0*n1/(n0+n1)*m'*inv(W)*m ;
F = (n0 + n1 - p -1)/(p*(n0+n1-2))*t2 ;
dof = n0 + n1 - p - 1 ;
 
if ( F > finv(1-alpha, p, dof) )
    equal = 0 ;
else
    equal = 1 ;
end




