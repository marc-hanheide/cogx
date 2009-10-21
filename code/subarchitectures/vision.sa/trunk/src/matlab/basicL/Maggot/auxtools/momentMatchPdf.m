%%
% Originally a part of: Maggot (developed within EU project CogX)
% Author: Matej Kristan, 2009 (matej.kristan@fri.uni-lj.si; http://vicos.fri.uni-lj.si/matejk/)
% Last revised: 2009
%%
function [new_mu, new_Cov, w_out] = momentMatchPdf(Mu, Cov, w)
% Collapse a mixture of Gaussians to a single Gaussian by moment matching
% [new_mu, new_Sigma] = momentMatchPdf(mu, Cov, w)
%
% w(i) - weight of i'th mixture component
% Mu(:,i), Cov{:,:,i} - params of i'th mixture component

% S = sum_c w_c (S_c + m_c m_c' + m m' - 2 m_c m') 
%   = sum_c w_c (S_c + m_c m_c') + m m' - 2 (sum_c m_c) m'
%   = sum_c w_c (S_c + m_c m_c') - m m'

if isempty(w)
    new_mu = [] ;
    new_Cov = [] ;
    w_out = [] ;
    return ;
end

w_out = [] ;
sumw = sum(w) ;
w = w/sumw ;
new_mu = sum(Mu * diag(w), 2) ; % weighted sum of columns

n = length(new_mu) ;
new_Cov = zeros(n,n) ;
for j=1:length(w)
%  m = Mu(:,j) - new_mu ;
%  new_Cov = new_Cov + w(j) * (Cov{j} + m*m') ;  
    new_Cov = new_Cov + w(j)*( Cov{j} + Mu(:,j)*Mu(:,j)') ;
end  
new_Cov = new_Cov - new_mu*new_mu' ;

if nargout == 3
    w_out = sumw ;
end