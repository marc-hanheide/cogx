function [pdf_to_reduce, oneToOneCoherence, delta_mdl] = ...
          combineComponentsFullMDL( pdf_reference, pdf_to_reduce, addMDL ) 
      
if nargin < 3
    addMDL = 0 ;
end

pdf_reference.covariances = reshape(pdf_reference.covariances,1,1,length(pdf_reference.weights)) ;
pdf_to_reduce.covariances = reshape(pdf_to_reduce.covariances,1,1,length(pdf_to_reduce.weights)) ;

 
len = length( pdf_to_reduce.weights) ;
 
% initialize reduction matrices
intMatrix = initializeIntegralMatrix( pdf_reference, pdf_to_reduce ) ;

change = 1 ;
while change > 0
       change = 0 ;
    [pair, H_min] = getMinimumDistancePair( intMatrix.H ) ;
    if ( isempty(pair) ) change = 0 ; break ; end
%      [ pdf_to_reduce, compress_this, intMatrix ] = tryToCombine( pdf_reference, pdf_to_reduce, pair, intMatrix ) ;
    [ compress_this, intMatrix, pdf_to_reduce ] =...
         combinePairOfComponents( pdf_reference, pdf_to_reduce, intMatrix, pair ) ;    
    if ( compress_this > 0 ) change = 1 ; end
    if( length(pdf_to_reduce.weights) < 1 ) change = 0 ; end ; 
    
    
end

 

function [pair, H_min] = getMinimumDistancePair( H )

H_min = sum(sum(H)) ;
pair = [1 1] ;
len = rows(H) ;
for i = 1 : len
    for j = i + 1 : len 
        if ( H(i,j) < H_min )
            pair = [i, j] ;
            H_min = H(i,j) ; 
        end
    end
end

% ------------------------------------------------------ %
function intMatrix = initializeIntegralMatrix( pdf_reference, pdf_reduced )

% horizontal ... reference
% vertical   ... compressed
P = zeros(length(pdf_reduced.weights),length(pdf_reference.weights)) ;
for i = 1 : length(pdf_reduced.weights)
    w1 = pdf_reduced.weights(i) ;
    mu1 = pdf_reduced.mu(:,i) ;
    C1 =  pdf_reduced.covariances(:,:,i) ;
    for j = 1 : length(pdf_reference.weights)
        w2 = pdf_reference.weights(j) ;
        mu2 = pdf_reference.mu(:,j) ;
        C2 = pdf_reference.covariances(:,:,j) ;

        p_ji = normpdf(mu1, mu2, [], C1 + C2 ) ;
        P(i,j) = p_ji ;
    end
end

% hellinger among pairs of components
H = zeros(length(pdf_reduced.weights),length(pdf_reference.weights)) ;
for i = 1 : length(pdf_reduced.weights)
    w1 = pdf_reduced.weights(i) ;
    mu1 = pdf_reduced.mu(:,i) ;
    C1 =  pdf_reduced.covariances(:,:,i) ;
    for j =i+1 : length(pdf_reduced.weights)
        w2 = pdf_reduced.weights(j) ;
        mu2 = pdf_reduced.mu(:,j) ;
        C2 = pdf_reduced.covariances(:,:,j) ;       
        H(i,j) = hellinger2Norm( w1, mu1, C1, w2, mu2, C2 ) ;        
    end
end

W = repmat(pdf_reduced.weights',1,length(pdf_reference.weights)) ; % reference weight matrix
N = (pdf_reference.weights)*pdf_reference.N ; % expected number of data-points

intMatrix.P = P.*W ; % P matrix
intMatrix.N = N ;
intMatrix.N_samples = pdf_reference.N ;
intMatrix.H = H ;


% ------------------------------------------------------ %
function mdl = getMdlUnderPdf( pdf_reference, pdf_reduced )

intMatrix = initializeIntegralMatrix( pdf_reference, pdf_reduced ) ;
mdl = mdlFromIntMatrix( intMatrix, pdf_reference, pdf_reduced ) ;

% ------------------------------------------------------ %
function mdl = mdlFromIntMatrix( intMatrix, pdf_reference, pdf_reduced ) 

compress_this = 0 ;
 
Cw = intMatrix.P ;
N = intMatrix.N ;

% expected log-likelihood for full distribution
Lf = sum(log2(sum(Cw,1)).*N) ;
 
mdl = getMDL( rows(pdf_reduced.mu), length(pdf_reduced.weights), Lf, intMatrix.N_samples ) ;

% ------------------------------------------------------ %
function [ compress_this, intMatrix, pdf_reduced ] =...
         tryToRemoveComponents( pdf_reference, pdf_reduced, intMatrix, i, j, addMDL )

compress_this = 0 ;
% mask for components
s = ones(length(pdf_reduced.weights),1) ; 
s([i, j]) = 0 ;
S = repmat( s, 1, cols(intMatrix.P) ) ;
% mask generated

Cw = intMatrix.P ;
N = intMatrix.N ;

% expected log-likelihood for full distribution
Lf = sum(log2(sum(Cw,1)).*N) ;

% calculate the auxiliary component and integrals
w1 = pdf_reduced.weights(i) ;
mu1 = pdf_reduced.mu(:,i) ;
C1 = pdf_reduced.covariances(:,:,i) ;

w2 = pdf_reduced.weights(j) ;
mu2 = pdf_reduced.mu(:,j) ;
C2 = pdf_reduced.covariances(:,:,j) ;
 
w_a = [ w1 w2 ] ; w_a = w_a / sum(w_a) ;
mu = [mu1 , mu2 ] ; C = cat(3,C1,C2) ;
[mu0, C0] = momentMatching( w_a, mu, C ) ;
alpha0 = w1 + w2 ;
% auxiliary component calculated

% make auxiliary vector
K = zeros(1,cols(Cw)) ;
for ( k = 1 : length(K) )
    m2 = pdf_reference.mu(:,k) ;
    C2 = pdf_reference.covariances(:,:,k) ;
    K(k) = normpdf(mu0, mu2, [], C0 + C2 ) ;
end

% expected log-likelihood for compressed distribution
Lc = sum(log2(sum(Cw.*S,1) + K*alpha0).*N) ;

% Minimum Description Lengths
mdl_f = getMDL( rows(pdf_reduced.mu), length(pdf_reduced.weights), Lf, intMatrix.N_samples ) ;
mdl_c = getMDL( rows(pdf_reduced.mu), length(pdf_reduced.weights)-1, Lc, intMatrix.N_samples ) ;

disp(sprintf('Num comps %d: mdl=%1.3g; Num comps %d: mdl=%1.3g; Ratio: %1.3g',length(pdf_reduced.weights),mdl_f,length(pdf_reduced.weights)-1,mdl_c,abs(mdl_f - mdl_c)/abs(mdl_f) )) ,

% [mdl_f, mdl_c ]

if ( mdl_c + addMDL < mdl_f   )%&& abs(mdl_f - mdl_c)/abs(mdl_f) > 0.01
    compress_this = 1 ;
end
 

% if ( mdl_c + addMDL < mdl_f )
%     compress_this = 1 ;
% end

% if (mdl_f - mdl_c)/abs(mdl_f) > 0.1
%     compress_this = 1 ;
% end

% update compressed variables
if ( compress_this == 1 )
    id = find( s > 0 ) ;

    % compress P matrix
    P = Cw(id,:) ; P = [P; K*alpha0] ;
    intMatrix.P = P ; 
    
    % compress distribution
    mm = pdf_reduced.mu(:,id) ;
    cc = pdf_reduced.covariances(:,:,id) ;
    ww = pdf_reduced.weights(id) ;

    pdf_reduced.weights = [ww, alpha0] ;
    pdf_reduced.mu = [mm, mu0] ;
    pdf_reduced.covariances = cat(3,cc,C0) ;

    pdf_reduced.E = pdf_reduced.weights*pdf_reduced.N ;
end

% ------------------------------------------------------ %
function mdl = getMDL( D, M, L, N )

% M = length(pdf.weights) ;  number of components
% D = rows(pdf.mu) ;  number of dimensions

Ne = M - 1 + M*D + M*D*(D+1)/2 ;
mdl = (1/2)*Ne*log2(N) - L ;


% ------------------------------------------------------ %
function [m0, C0] = momentMatching( w, m, C )

d = rows(m) ;
m0 = sum(repmat(w,d,1).*m,2) ;
C0 = C(:,:,1)*0 ;
for i = 1 : length(w)
   C0 = C0 + w(i)*(C(:,:,i) + (m0-m(:,i))*(m0-m(:,i))') ;
end

% ------------------------------------------------------ %
% function dE = evalPairsIntegrals( w, mu, C, mu0, C0 ) 
% 
% tol = 1e-5* 1/length(w) ;
% 
% if ( sum(w < tol) > 1 )
%     dE = 10 ; return ;
% end
% 
% D = rows(mu) ;
% Nn = w*N ;
% w12 = sum(w) ;
% P = zeros(1,4) ;
% 
% mu1 = mu(:,1) ; mu2 = mu(:,2) ;
% w1 = w(1) ; w2 = w(2) ;
% C1 = C(:,:,1) ; C2 = C(:,:,2) ;
% 
% p11 = bhatta( mu1, C1, mu1, C1 ) ;
% p12 = bhatta( mu1, C1, mu2, C2 ) ;
% p22 = bhatta( mu2, C2, mu2, C2 ) ;
% p01 = bhatta( mu0, C0, mu1, C1 ) ;
% p02 = bhatta( mu0, C0, mu2, C2 ) ;
%  
% Es = (w(1)*p11 + w(2)*p12)^Nn(1) * (w(1)*p12 + w(2)*p22)^Nn(2) ;
% Em = (w12*p01)^Nn(1) * (w12*p02)^Nn(2) ;
% 
% ttol = 0 ;
% if ( Es <= ttol )
%     dE = 10 ; return ;
% end
% 
% if ( Em <= ttol )
%     dE = -10 ; return ;
% end
% 
% % dE = (1/4)*D*(D+1)*log2(sum(Nn)) - log2(Es) + log2(Em) ;
% Ns = 5 ; Nm = 2 ;
% dE = (1/2)*log(sum(Nn))*(Ns - Nm) + log2(Em) - log2(Es) ;
