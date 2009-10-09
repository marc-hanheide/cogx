function Sigma = MakeCovMatrix(mu,R)

d = size(mu,2);
k = size(mu,1); % nof components
Sigma = [] ;

for j = 1:k 
  % Cholesky triangular matrix of component's covariance matrix
  Rk = reshape(R(j,:),d,d);        
  Sj = Rk'*Rk ; 
  F = chol(Sj);
  Sigma = [Sigma; Sj] ;
end



