function [W,M,R,Tlogl] = em(X,T,kmax,nr_of_cand,plo,dia,pdf_reference)
% em - EM algorithm for adaptive multivariate Gaussian mixtures
%
%[W,M,R,Tlogl] = em(X,T,kmax,nr_of_cand,plo,dia)
%  X     - (n x d) d-dimensional zero-mean unit-variance data
%  T     - (m x d) test data (optional, set [] if none)
%  kmax  - maximum number of components allowed
%  nr_of_cand - number of candidates per component, zero gives non-greedy EM
%  plo   - if 1 then plot ellipses for 2-d data
%  dia   - if 1 then print diagnostics
%returns
%  W - (k x 1) vector of mixing weights
%  M - (k x d) matrix of components means
%  R - (k x d^2) matrix of Cholesky submatrices of components covariances
%      in vector reshaped format. To get the covariance of component k:
%      Rk = reshape(R(k,:),d,d); S = Rk'*Rk;
%  Tlogl -  average log-likelihood of test data
%
% Nikos Vlassis & Sjaak Verbeek, oct 2002
% see greedy-EM paper at http://www.science.uva.nl/~vlassis/publications

[n,d] = size(X); n1=ones(n,1);d1=ones(1,d);

Tlogl = [] ;
if d > 2 plo = 0; end
if isempty(T) test = 0;
else          test = 1;Tlogl=[];
end
if plo; figure(1);set(1,'Double','on');end
THRESHOLD = 1e-5;

if nr_of_cand 
  k = 1;  if dia; fprintf('Greedy ');end
else 
  k = kmax;
  if dia; fprintf('Non-greedy ');end
end

if ( nargin < 7 )
    pdf_reference = [] ;
end

if dia fprintf('EM initialization\n'); end
[W,M,R,P,sigma] = em_init_km(X,k,0,pdf_reference);
sigma=sigma^2;

oldlogl = -realmax;

while 1
  % apply EM steps to the complete mixture until convergence
  if dia     fprintf('EM steps');  end
  while 1
    [W,M,R] = em_step(X,W,M,R,P,plo);
    if dia       fprintf('.');     end
    % likelihoods L (n x k) for all inputs and all components
    L = em_gauss(X,M,R);
    % mixture F (n x 1) and average log-likelihood
    F = L * W;
    F(find(F < realmin)) = realmin;
    logl = mean(log(F)); 
    
    % posteriors P (n x k) and their sums
    P = L .* (ones(n,1)*W')  ./ (F*ones(1,k));
        
    if abs(logl/oldlogl-1) < THRESHOLD
      if dia         fprintf('\n');        fprintf('Logl = %g\n', logl);end
      break;
    end
    oldlogl = logl;
  end

 if test % average log-likelihood of test set
      Ft = em_gauss(T,M,R) * W;
      Ft(find(Ft < eps)) = eps;
      Tlogl = [Tlogl; mean(log(Ft))];
 else
        Tlogl=0;
 end
 
  if k == kmax;    return;  end

  if dia    fprintf('Trying component allocation');  end
  [Mnew,Rnew,alpha] = rand_split(P,X,M,R,sigma,F,W,nr_of_cand); 
  if alpha==0
    if test % average log-likelihood of test set
      Ft = em_gauss(T,M,R) * W;
      Ft(find(Ft < eps)) = eps;
      Tlogl = [Tlogl; mean(log(Ft))];
    else
      Tlogl=0; 
    end  
    return;
  end
  K                 = em_gauss(X,Mnew,Rnew);
  PP                = F*(1-alpha)+K*alpha;
  LOGL              = mean(log(PP));

  % optimize new mixture with partial EM steps updating only Mnew,Rnew
  veryoldlogl = logl; oldlogl = LOGL;done_here=0;
  Pnew = (K.*(ones(n,1)*alpha))./PP;
  while ~done_here
    if dia         fprintf('*');    end
    [alpha,Mnew,Rnew] = em_step(X,alpha,Mnew,Rnew,Pnew,0);
    K    = em_gauss(X,Mnew,Rnew); Fnew = F*(1-alpha)+K*alpha;
    Pnew = K*alpha./Fnew;         logl = mean(log(Fnew));
    if abs(logl/oldlogl-1)<THRESHOLD done_here=1;end
    oldlogl=logl;
  end
  % check if log-likelihood increases with insertion
  if logl <= veryoldlogl
    if dia fprintf('Mixture uses only %d components\n', k);end
    if test % average log-likelihood of test set
      Ft = em_gauss(T,M,R) * W; Ft(find(Ft < eps)) = eps;
      Tlogl = [Tlogl; mean(log(Ft))];
    else Tlogl=0; end
    return;
  end
  % allocate new component
  M = [M; Mnew];
  R = [R; Rnew];
  W = [(1-alpha)*W; alpha];
  k = k + 1;
  if dia   fprintf(' k = %d\n', k);fprintf('LogL = %g\n', logl);end
  % prepare next EM step
  L = em_gauss(X,M,R);
  F = L * W;F(find(F<realmin))=realmin;
  P = L .* (ones(n,1)*W')  ./ (F*ones(1,k));
end
    
