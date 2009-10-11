function [ pdf_n, oneToOneCoherence ] = combineComponents( pdf_n ) 

len = length( pdf_n.weights) ;

if ( pdf_n.N == 10 ) 
    sdf = 56 ;
end

oneToOneCoherence = 1 ;
i = 1 ; pdf_v = pdf_n ;
% try to combine split pairs first
for steps = 1 : len/2   
%     [pdf_n, changeX] = tryToCombineUnderHellinger( pdf_n, i, i+1 ) ;
    [pdf_n, changeX] = tryToCombine( pdf_n, i, i+1 ) ;
    if ( changeX == 0 ) i = i + 2 ; oneToOneCoherence = 0 ; end
end



% pdf_n = resortComponents( pdf_n ) ;
%  pdf_n
%  return
change = 1 ;
while change > 0
    changeX = 0 ;
    change = 0 ;
    len = length( pdf_n.weights ) ;
    for i = 1 : len
        for j = i + 1 : len 
           if ( i == j ) continue ; end  
%            [pdf_n, changeX] = tryToCombineUnderHellinger( pdf_n, i, i+1 ) ;
           [pdf_n, changeX] = tryToCombine( pdf_n, i, j ) ; 
           change = change + changeX ;
           if changeX > 0 oneToOneCoherence = 0 ; break ; end
        end
        if changeX > 0 break ; end
    end
end


% figure(2) ; plot(1:length(pdf_n.weights),pdf_n.weights) ; drawnow ;

% ------------------------------------------------------ %
function pdf = resortComponents( pdf )

idx = [length(pdf.weights):-1:1];
pdf.weights = pdf.weights(idx) ;
pdf.mu = pdf.mu(:,idx) ;
pdf.covariances = pdf.covariances(:,:,idx) ;
pdf.E = pdf.E(idx) ;



function [ pdf_n, combined ] = tryToCombine( pdf_n, i, j )

tol = 1e-10 ;

N = pdf_n.N ;
w1 = pdf_n.weights(i) ;
mu1 = pdf_n.mu(:,i) ;
C1 = pdf_n.covariances(:,:,i) ;

w2 = pdf_n.weights(j)*(i ~= j ) ;
mu2 = pdf_n.mu(:,j) ;
C2 = pdf_n.covariances(:,:,j) ;
 
w_a = [ w1 w2 ] ; w_a = w_a / sum(w_a) ;
mu = [mu1 , mu2 ] ;
C = cat(3,C1,C2) ;
[mu0, C0] = momentMatching( w_a, mu, C ) ;

 w = [ w1, w2 ] ;
if ( det(C1) < tol || det(C2) < tol || w1 < tol || w2 < tol )
    combined = 1 ;
else   
    dE = evalPairsIntegrals( w, mu, C, mu0, C0, N ) ;
    combined = dE > 0 ;
end

mm = [] ; cc = [] ; ww = [] ;
if ( combined == 1 )
    len = length(pdf_n.weights) ;
    sc = ones(1,len) ; sc(i) = 0 ; sc(j) = 0 ;
    id = find( sc > 0 ) ;  
    
    mm = pdf_n.mu(:,id) ;
    cc = pdf_n.covariances(:,:,id) ;
    ww = pdf_n.weights(id) ;
    
    pdf_n.weights = ww ;
    pdf_n.mu = mm ;
    pdf_n.covariances = cc ;
    
    pdf_n.weights = [pdf_n.weights, sum(w)] ;
    pdf_n.mu = [pdf_n.mu, mu0] ; 
    pdf_n.covariances = cat(3,pdf_n.covariances,C0) ;

    pdf_n.E = pdf_n.weights*N ;
end


% ------------------------------------------------------ %
function [ pdf_n, combined ] = tryToCombineUnderHellinger( pdf_n, i, j )

tol = 1e-10 ;

N = pdf_n.N ;
w1 = pdf_n.weights(i) ;
mu1 = pdf_n.mu(:,i) ;
C1 = pdf_n.covariances(:,:,i) ;

w2 = pdf_n.weights(j)*(i ~= j ) ;
mu2 = pdf_n.mu(:,j) ;
C2 = pdf_n.covariances(:,:,j) ;
 
w_a = [ w1 w2 ] ; w_a = w_a / sum(w_a) ;
mu = [mu1 , mu2 ] ;
C = cat(3,C1,C2) ;
[mu0, C0] = momentMatching( w_a, mu, C ) ;
w0 = w1 + w2 ;

pdf_c = pdf_n ;
combined = 1 ; 
mm = [] ; cc = [] ; ww = [] ;
if ( combined == 1 )
    len = length(pdf_n.weights) ;
    sc = ones(1,len) ; sc(i) = 0 ; sc(j) = 0 ;
    id = find( sc > 0 ) ;  
    
    mm = pdf_n.mu(:,id) ;
    cc = pdf_n.covariances(:,:,id) ;
    ww = pdf_n.weights(id) ;
 
    pdf_c.weights = [ww, w0] ;
    pdf_c.mu = [mm, mu0] ; 
    pdf_c.covariances = cat(3,cc,C0) ;

    pdf_c.E = pdf_n.weights*N ;
end

pdf_n.covariances = reshape(pdf_n.covariances, size(pdf_n.covariances,3),1 ) ;
pdf_c.covariances = reshape(pdf_c.covariances, size(pdf_c.covariances,3),1 ) ;

H = uHellingerJointSupport(  pdf_c, pdf_n ) ;
% disp(sprintf('Hellinger distance: %1.3g', H )) ;

combined = 0 ;
if ( H < 0.05 ) pdf_n = pdf_c ; combined = 1 ; end
pdf_n.covariances = reshape(pdf_n.covariances, 1, 1, size(pdf_n.covariances,1) ) ;
 
% ------------------------------------------------------ %
function P = bhatta_og( mu1, C1, mu2, C2 )

D = rows(mu1) ;
C = inv(inv(C1) + inv(C2)) ;
mu = C*( inv(C1)*mu1 + inv(C2)*mu2 ) ;
K = mu1*inv(C1)*mu1' + mu2*inv(C2)*mu2' - mu*inv(C)*mu' ;

P = ((2*pi)^(D/2)*sqrt(det(C1*C2*C)))^(-1) * exp(-K/2) ;
 
% ------------------------------------------------------ %
function P = bhatta( mu1, C1, mu2, C2 )

P =  normpdf(mu1, mu2, [], C1 + C2 ) ;

% D = rows(mu1) ;
% C = inv(C1 + C2) ;
% mu = mu1 - mu2 ;
% P = ( 1/(sqrt(2*pi)) )*sqrt(det(C)) * exp(-0.5 *mu'*C*mu) ;
 
% ------------------------------------------------------ %
% ------------------------------------------------------ %
function [m0, C0] = momentMatching( w, m, C )

d = rows(m) ;
m0 = sum(repmat(w,d,1).*m,2) ;
C0 = C(:,:,1)*0 ;
for i = 1 : length(w)
   C0 = C0 + w(i)*(C(:,:,i) + (m0-m(:,i))*(m0-m(:,i))') ;
end
