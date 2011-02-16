function [pdf_to_reduce ] = getMDLoptimalDistribution( pdf_reference, pdf_to_reduce ) 
      
pdf_reference.covariances = reshape(pdf_reference.covariances,1,1,length(pdf_reference.weights)) ;
pdf_to_reduce.covariances = reshape(pdf_to_reduce.covariances,1,1,length(pdf_to_reduce.weights)) ;
pdf_to_reduce

len = length( pdf_to_reduce.weights) ;
 
% initialize reduction matrices
intMatrix = initializeIntegralMatrix( pdf_reference, pdf_to_reduce ) ;
 
change = 1 ;
while change > 0
    change = 0 ;
    [pair, H_min] = getMinimumDistancePair( intMatrix.H ) ;

    if ( isempty(pair) ) change = 0 ; break ; end
%     if ( H_min > 0.1 ) break ; end
    
%      [ pdf_to_reduce, compress_this, intMatrix ] = tryToCombine( pdf_reference, pdf_to_reduce, pair, intMatrix ) ;
    % is working now
    [ pdf_to_reduce, compress_this, intMatrix ] = tryToCombine_HellingerMDL( pdf_reference, pdf_to_reduce, pair, intMatrix ) ;
     
     
     
%     [ compress_this, intMatrix, pdf_to_reduce ] =...
%          combinePairOfComponents( pdf_reference, pdf_to_reduce, intMatrix, pair ) ;    
    if ( compress_this > 0 ) change = 1 ; end
    if( length(pdf_to_reduce.weights) < 1 ) change = 0 ; end ;
end

pdf_to_reduce.covariances = reshape(pdf_to_reduce.covariances, length(pdf_to_reduce.weights), 1, 1 ) ;





%------------------------------------------------------------------------ %
function [ pdf_n, combined, intMatrix ] = tryToCombine_HellingerMDL( pdf_reference, pdf_n, pair, intMatrix )

  
[pdf_new , new_ownerships, id_remains ] = remakeDistribution(intMatrix, pair, pdf_reference, pdf_n ) ;
 
 

disp('Is moment matching actually more appropriate at this stage in MDL?') ;
% dE = evalSavingMDL( fr_0.weights, fr_0.mu, fr_0.covariances, fd_0.weights, fd_0.mu, fd_0.covariances, N ) ;
% mdl_full = evaluateHypothesis( pdf_reference, pdf_n ) ;
% mdl_com = evaluateHypothesis( pdf_reference, pdf_new ) ;
dE = getHellMDL( pdf_reference, pdf_n, pdf_new ) ;

combined = dE > 0 ;

mm = [] ; cc = [] ; ww = [] ;
if ( combined == 1 )
    N = pdf_n.N ;
    pdf_n = pdf_new ;

    pdf_n.N = N ;
    pdf_n.E = pdf_n.weights*pdf_n.N ;
 
    % update ownerships
    intMatrix.ownerships = new_ownerships ;    
    
    % update Hellinger matrix
    intMatrix.H = intMatrix.H(id_remains,id_remains) ;
    w0 = pdf_n.weights(length(pdf_n.weights)) ;
    m0 = pdf_n.mu(:,length(pdf_n.weights))
    C0 = pdf_n.covariances(:,:,length(pdf_n.weights)) ;
    h = getHellingerAlltoOne( pdf_n, w0, m0, C0 ) ;
    
    
   
    intMatrix.H = [[intMatrix.H, h(1:length(h)-1)]; h'] ;
end


%------------------------------------------------------------------------ %
function H = getHellError( pdf_n )


[m0, C0] = momentMatching( pdf_n.weights, pdf_n.mu, pdf_n.covariances ) ;
w0 = sum(pdf_n.weights) ; 
% pdf_n.covariances = reshape(pdf_n.covariances,length(pdf_n.weights),1) ;

f2.mu = m0 ;
f2.covariances = C0 ;
f2.weights = w0 ;

pdf_n.covariances = reshape(pdf_n.covariances,length(pdf_n.weights),1) ;
H = uHellingerJointSupport( pdf_n, f2 ) ;
 

%------------------------------------------------------------------------ %
function mdl = getHellMDL( pdf_reference, pdf_n, pdf_com )

pdf_reference.covariances = reshape(pdf_reference.covariances,length(pdf_reference.weights),1) ;
pdf_n.covariances = reshape(pdf_n.covariances,length(pdf_n.weights),1) ;
pdf_com.covariances = reshape(pdf_com.covariances,length(pdf_com.weights),1) ;

% H = uHellingerJointSupport( f1, f2 ) ;
H_n = uHellinger( pdf_reference, pdf_n ) ;
H_com = uHellinger( pdf_reference, pdf_com ) ;

n_ref = length(pdf_reference.weights) ;
n_n = length(pdf_n.weights) ;
n_c = length(pdf_com.weights) ;

pdf_ref0 = inflateReferenceToAlpha( pdf_reference ) ;
% lam = uHellinger( pdf_reference, pdf_ref0 ) / 3;
% t = 0.1 ; 
% K1 = 32 ;
% K2 = log2(exp(1)*lam^2*2*pi/t^2)/lam^2 ;
% k2 = K2 / K1 ;


% disp('Problem with adaptive threshold is that it brakes at N=100000000...')
lam = uHellinger( pdf_reference, pdf_ref0 )/10 ;
t = 0.01 ; 
c = lam/t ; if( c < 1) c = 1 ; end
lam = c*t ;
K1 = 32 ;
K2 = log2(exp(1)*c)/lam ;
k2 = K2 / K1 ;

% k2 = 50 ;

% if (pdf_reference.N > 117)
%     sfdg =6
% end

Fn = n_ref - n_n - k2*H_n ;
Fc = n_ref - n_c - k2*H_com ;

mdl = Fc - Fn ;

% d = rows(pdf_reference.mu) ; 
% M_n = length(pdf_com.weights) ;
% N_n = M_n-1 + M_n*d + M_n*d*(d+1)/2 ;
% 
% M_com = length(pdf_com.weights) ;
% N_com = M_com-1 + M_com*d + M_com*d*(d+1)/2 ;
% 
% mdl_n = (1/2)*N_n*log2(sum(pdf_reference.N)) - L ;
% mdl_com = (1/2)*N_com*log2(sum(pdf_reference.N)) - L ;


%------------------------------------------------------------------------ %
function pdf_ref = inflateReferenceToAlpha( pdf_ref )

 
for i = 1 : length(pdf_ref.weights)
    N = max([round(pdf_ref.N*pdf_ref.weights(i)), 2]) ;
    bet = (N - 1)/2 ;
    alph = (N-1)/( 2* pdf_ref.covariances(i,:)) ;
    pdf_ref.covariances(i,:) = gaminv(0.95,bet+1,alph^(-1)) ;
end



%------------------------------------------------------------------------ %
function [ pdf_n, combined, intMatrix ] = tryToCombineMy( pdf_reference, pdf_n, pair, intMatrix )

  
[pdf_new , new_ownerships, id_remains ]= remakeDistribution(intMatrix, pair, pdf_reference, pdf_n ) ;
 
 

disp('Is moment matching actually more appropriate at this stage in MDL?') ;
% dE = evalSavingMDL( fr_0.weights, fr_0.mu, fr_0.covariances, fd_0.weights, fd_0.mu, fd_0.covariances, N ) ;
mdl_full = evaluateHypothesis( pdf_reference, pdf_n ) ;
mdl_com = evaluateHypothesis( pdf_reference, pdf_new ) ;
dE = mdl_full - mdl_com ;

combined = dE > 0 ;

mm = [] ; cc = [] ; ww = [] ;
if ( combined == 1 )
    N = pdf_n.N ;
    pdf_n = pdf_new ;

    pdf_n.N = N ;
    pdf_n.E = pdf_n.weights*pdf_n.N ;
 
    % update ownerships
    intMatrix.ownerships = new_ownerships ;    
    
    % update Hellinger matrix
    intMatrix.H = intMatrix.H(id_remains,id_remains) ;
    w0 = pdf_n.weights(length(pdf_n.weights)) ;
    m0 = pdf_n.mu(:,length(pdf_n.weights))
    C0 = pdf_n.covariances(:,:,length(pdf_n.weights)) ;
    h = getHellingerAlltoOne( pdf_n, w0, m0, C0 ) ;
    intMatrix.H = [[intMatrix.H, h(1:length(h)-1)]; h'] ;
end


function [ pdf_new, new_ownerships,id_remains ] = remakeDistribution(intMatrix, pair, pdf_reference, pdf_n ) 

% calculate the auxiliary component and integrals
ownerships_current = [intMatrix.ownerships{pair(1)},intMatrix.ownerships{pair(2)}] ;
id = ownerships_current ;
fr_0.mu = pdf_reference.mu(:,id) ;        
fr_0.weights = pdf_reference.weights(id) ;        
fr_0.covariances = pdf_reference.covariances(:,:,id) ; 

% id = pair ;
% fr_0.mu = pdf_n.mu(:,id) ;        
% fr_0.weights = pdf_n.weights(id) ;        
% fr_0.covariances = pdf_n.covariances(:,:,id) ;               

[m0, C0] = momentMatching( fr_0.weights, fr_0.mu, fr_0.covariances ) ;

fd_0.weights = sum(fr_0.weights) ;        
fd_0.mu = m0 ;        
fd_0.covariances = C0 ;

fd_0 = refitGaussianMixtureFunApproxSingle( fd_0, fr_0 ) ;
%fr_0.weights = fd_0.weights*fr_0.weights/sum(fr_0.weights)  ;

sc = ones(1,length(pdf_n.weights)) ; sc(pair) = 0 ; id = find(sc > 0) ;
id_remains = id ;

% select components from 
pdf_new.mu = pdf_n.mu(:,id) ;
pdf_new.weights = pdf_n.weights(id) ;
pdf_new.covariances = pdf_n.covariances(:,:,id) ;
new_ownerships = intMatrix.ownerships(id) ;

pdf_new.mu = [pdf_new.mu, fd_0.mu ] ;
pdf_new.weights = [pdf_new.weights, fd_0.weights ] ; 
pdf_new.weights = pdf_new.weights / sum(pdf_new.weights) ;
pdf_new.covariances = cat(3, pdf_new.covariances,  fd_0.covariances ) ;
new_ownerships = vertcat(new_ownerships, {ownerships_current}) ;

function mdl_f = evaluateHypothesis( pdf_ref, pdf_com )

Nref = round(pdf_ref.weights*pdf_ref.N) ;
len_ref = length(pdf_ref.weights) ;
len_com = length(pdf_com.weights) ;

L = 0 ; Lx = 0 ;
% for each owner, calculate the 
for i = 1 : len_ref
    mu1 = pdf_ref.mu(:,i) ;
    C1 = pdf_ref.covariances(:,:,i) ;
    p_tmp = 0 ;
    for j = 1 : len_com
        mu2 = pdf_com.mu(:,j) ;
        C2 = pdf_com.covariances(:,:,j) ;
        
        p_ij = normpdf(mu1, mu2, [], C1 + C2 ) ; 
        p_tmp = p_tmp + p_ij*pdf_com.weights(j) ;        
    end
    Li = p_tmp^Nref(i) ;    
%     L = L - log2(p_tmp)*Nref(i) ; %log2(Li) ;
    
   % Lx = Lx + (1-p_tmp)*Nref(i)/log(2) ;
    Lx = Lx + (1-p_tmp)*Nref(i) ;
end

% Lx = Lx / sum(Nref)

d = rows(mu1) ; M = length(pdf_com.weights) ;
Ns = M-1 + M*d + M*d*(d+1)/2 ;

a = -0.5 ;
mdl_f = 0.5*Lx - (1/2)*Ns*log2(sum(pdf_ref.N)) %(1/2)*Ns*log2(sum(pdf_ref.N)) - a*Lx ;



function dE = evalSavingMDL( w, mu, C, w0, mu0, C0, N ) 

tol = 1e-5* 1/length(w) ;

if ( sum(w < tol) > 1 )
    dE = 10 ; return ;
end

D = rows(mu) ;
Nn = round(w*N) ;
 
% ss = sum(w) ;
% w12 = 1 ; w = w / ss ;
% w12 = w0/ ss ;%sum(w) ;
w12 = w0  ;%sum(w) ;

P = zeros(1,4) ;

mu1 = mu(:,1) ; mu2 = mu(:,2) ;
w1 = w(1) ; w2 = w(2) ;
C1 = C(:,:,1) ; C2 = C(:,:,2) ;

p11 = bhatta( mu1, C1, mu1, C1 ) ;
p12 = bhatta( mu1, C1, mu2, C2 ) ;
p22 = bhatta( mu2, C2, mu2, C2 ) ;
p01 = bhatta( mu0, C0, mu1, C1 ) ;
p02 = bhatta( mu0, C0, mu2, C2 ) ;
 
f0.mu = mu0 ; f0.weights = w0 ; f0.covariances = C0(1) ;
f12.mu = mu ; f12.weights = w ; f12.covariances = [C(1);C(2) ];

Es = (w(1)*p11 + w(2)*p12)^Nn(1) * (w(1)*p12 + w(2)*p22)^Nn(2) ;
Em = (w12*p01)^Nn(1) * (w12*p02)^Nn(2) ;

ttol = 0 ;
if ( Es <= ttol )
    dE = -10 ; return ;
end

if ( Em <= ttol )
    dE = -10 ; return ;
end


dE = mdlCriterion ( Es, Em, Nn ) ;  

% ------------------------------------------------------------- %
function [ pdf_n, combined, intMatrix ] = tryToCombine( pdf_reference, pdf_n, pair, intMatrix )

i = pair(1) ;
j = pair(2) ; 
tol = 1e-10 ;

N = pdf_n.N ;
w1 = pdf_n.weights(i) ;
mu1 = pdf_n.mu(:,i) ;
C1 = pdf_n.covariances(:,:,i) ;

w2 = pdf_n.weights(j)*(i ~= j ) ;
mu2 = pdf_n.mu(:,j) ;
C2 = pdf_n.covariances(:,:,j) ;
 
% calculate the auxiliary component and integrals
ownerships_current = [intMatrix.ownerships{pair(1)},intMatrix.ownerships{pair(2)}] ;
% id = ownerships_current ;
% fr_0.mu = pdf_reference.mu(:,id) ;        
% fr_0.weights = pdf_reference.weights(id) ;        
% fr_0.covariances = pdf_reference.covariances(:,:,id) ; 
id = pair ;
fr_0.mu = pdf_n.mu(:,id) ;        
fr_0.weights = pdf_n.weights(id) ;        
fr_0.covariances = pdf_n.covariances(:,:,id) ;               

[m0, C0] = momentMatching( fr_0.weights, fr_0.mu, fr_0.covariances ) ;

fd_0.weights = sum(fr_0.weights) ;        
fd_0.mu = m0 ;        
fd_0.covariances = C0 ;

fd_0 = refitGaussianMixtureFunApproxSingle( fd_0, fr_0 ) ;
% fr_0.weights = fd_0.weights*fr_0.weights/sum(fr_0.weights)  ;
 
alpha0 = fd_0.weights ; %sum(fd_0.weights) ;
% auxiliary component calculated

w = [ w1, w2 ] ;
if ( det(C1) < tol || det(C2) < tol || w1 < tol || w2 < tol )
    combined = 1 ;
else 
%     len = length(pdf_n.weights) ; sc = ones(1,len) ; sc(i) = 0 ; sc(j) = 0 ; id = find( sc > 0 ) ;
%     mm = pdf_n.mu(:,id) ; cc = pdf_n.covariances(:,:,id) ; ww = pdf_n.weights(id) ;
%     % regularize weights
%     pdf_N.weights = [ww, alpha0] ;  pdf_N.weights = pdf_N.weights / sum(pdf_N.weights) ;
%     alp = pdf_N.weights(length(pdf_N.weights)) ; pdf_N.mu = [mm, m0] ; pdf_N.covariances = cat(3,cc,C0) ;
%     mdl_full = getMdlUnderPdf( pdf_reference, pdf_reference ) ;
%     mdl_reduced = getMdlUnderPdf( pdf_reference, pdf_N ) ;
%     dE = mdl_full - mdl_reduced ;
   dE = evalPairsMDL( fr_0.weights, fr_0.mu, fr_0.covariances, fd_0.weights, fd_0.mu, fd_0.covariances, N ) ;
    combined = dE > 0 ;
end

% alpha = 0.001 ;
% combined = testForEqualityOfComponents( w1, mu1, C1, w2, mu2, C2, N, alpha ) ;


mm = [] ; cc = [] ; ww = [] ;
if ( combined == 1 )
    len = length(pdf_n.weights) ;
    sc = ones(1,len) ; sc(i) = 0 ; sc(j) = 0 ;
    id = find( sc > 0 ) ;  
 
    % compress distribution
    mm = pdf_n.mu(:,id) ;
    cc = pdf_n.covariances(:,:,id) ;
    ww = pdf_n.weights(id) ;

    % regularize weights
    pdf_n.weights = [ww, alpha0] ; 
    pdf_n.weights = pdf_n.weights / sum(pdf_n.weights) ;
    alpha0 = pdf_n.weights(length(pdf_n.weights)) ;
    
    pdf_n.mu = [mm, m0] ;
    pdf_n.covariances = cat(3,cc,C0) ;

    pdf_n.E = pdf_n.weights*pdf_n.N ;
 
    
        % update ownerships
    intMatrix.ownerships = vertcat(intMatrix.ownerships(id),{ownerships_current}) ;    
    
    % update Hellinger matrix
    intMatrix.H = intMatrix.H(id,id) ;
    h = getHellingerAlltoOne( pdf_n, alpha0, m0, C0 ) ;
    intMatrix.H = [[intMatrix.H, h(1:length(h)-1)]; h'] ;
end


function dE = evalPairsMDL( w, mu, C, w0, mu0, C0, N ) 

tol = 1e-5* 1/length(w) ;

if ( sum(w < tol) > 1 )
    dE = 10 ; return ;
end

D = rows(mu) ;
Nn = round(w*N) ;

% ss = sum(w) 
% w = w/ss ; w0 = w0 / ss ;
 
% ss = sum(w) ;
% w12 = 1 ; w = w / ss ;
% w12 = w0/ ss ;%sum(w) ;
w12 = w0  ;%sum(w) ;

P = zeros(1,4) ;

mu1 = mu(:,1) ; mu2 = mu(:,2) ;
w1 = w(1) ; w2 = w(2) ;
C1 = C(:,:,1) ; C2 = C(:,:,2) ;

p11 = bhatta( mu1, C1, mu1, C1 ) ;
p12 = bhatta( mu1, C1, mu2, C2 ) ;
p22 = bhatta( mu2, C2, mu2, C2 ) ;
p01 = bhatta( mu0, C0, mu1, C1 ) ;
p02 = bhatta( mu0, C0, mu2, C2 ) ;
 
f0.mu = mu0 ; f0.weights = w0 ; f0.covariances = C0(1) ;
f12.mu = mu ; f12.weights = w ; f12.covariances = [C(1);C(2) ];

Nx = 1 ;
Nx = sum(Nn) ;
Nsup = Nn / Nx ;

% Es = (w(1)*p11 + w(2)*p12)^Nn(1) * (w(1)*p12 + w(2)*p22)^Nn(2) ;
% Em = (w12*p01)^Nn(1) * (w12*p02)^Nn(2) ;
Es = (w(1)*p11 + w(2)*p12)^Nsup(1) * (w(1)*p12 + w(2)*p22)^Nsup(2) ;
Em = (w12*p01)^Nsup(1) * (w12*p02)^Nsup(2) ;
dE = mdlCriterion ( Es, Em, Nn, Nx ) 

% approximate loglikelihood
% l_Es = -((1-(w(1)*p11 + w(2)*p12))/log(2)*Nn(1) + (1-(w(1)*p12 + w(2)*p22))/log(2)*Nn(2)) ;
% l_Em = -((1-(w12*p01))/log(2)*Nn(1) + (1-(w12*p02))/log(2)*Nn(2)) ;
% dE = mdlCriterionLog ( l_Es*10, l_Em*10, Nn ) 

% l_Es = log2((w(1)*p11 + w(2)*p12))*Nn(1) + log2(w(1)*p12 + w(2)*p22)*Nn(2) ;
% l_Em = log2(w12*p01)*Nn(1) + log2(w12*p02)*Nn(2) ;
% ttol = 0 ;
% if ( Es <= ttol )
%     dE = -10 ; return ;
% end
% 
% if ( Em <= ttol )
%     dE = -10 ; return ;
% end


% dE = mdlCriterion ( Es, Em, Nn, Nx ) 
%  dfg = 67 ;

 
 
function dE = mdlCriterionLog ( l_Es, l_Em, Nn )
 
% mdl criterion
Ns = 2 - 1 +2*1 + 2*2/2 ; Nm = 1 - 1 + 1 + 1*2/2 ;
mdl_f = (1/2)*Ns*log2(sum(Nn)) - (l_Es) ;
mdl_c = (1/2)*Nm*log2(sum(Nn)) - (l_Em) ;
% [ mdl_c, mdl_f ]
% D = 1 ;

% disp(sprintf('Num comps %d: mdl=%1.3g; Num comps %d: mdl=%1.3g; Ratio:
% %1.3g',length(pdf_reduced.weights),mdl_f,length(pdf_reduced.weights)-1,mdl_c,abs(mdl_f - mdl_c)/abs(mdl_f) )) ,
% dE = (1/4)*D*(D+1)*log2(sum(Nn)) - log2(Es) + log2(Em) ;
% dE = (1/2)*log2(sum(Nn))*(Ns - Nm) +  log2(Em) - 0.7*log2(Es) ;
dE = (1/2)*log2(sum(Nn))*(Ns - Nm) + ((l_Em) - (l_Es)) ;

function dE = mdlCriterion ( Es, Em, Nn,Nx )
% mdl criterion
Ns = 2 - 1 +2*1 + 2*2/2 ; Nm = 1 - 1 + 1 + 1*2/2 ;
if ( Em == 0 && Es == 0 ) 
    dE = -1 ; return;
end

dE = (1/2)*log2(sum(Nn))*(Ns - Nm) + (log2(Em) - log2(Es))*Nx ;


function P = bhatta( mu1, C1, mu2, C2 )

P =  normpdf(mu1, mu2, [], C1 + C2 ) ;
% 

% --------------------------------------------------------------------- %
function f_dest = reEstimateParameters( f_ref, f_dest, ownerships )
len_refs = length(f_ref.weights) ;
len_dest = length(f_dest.weights) ;
d = rows(f_dest.mu) ;
f_ref.covariances = reshape(f_ref.covariances',d,d,len_refs) ;
f_dest.covariances = reshape(f_dest.covariances',d,d,len_dest) ;


hisNums = zeros(1,len_dest) ;
valid = ones(1,len_dest) ;
len_dest = length(f_dest.weights) ;
    for j = 1 : len_dest
        f0.weights = f_dest.weights ;
        
        
        id = find(ownerships==j) ;
        if isempty(id)
            f_dest.weight(j) = 0 ;
            continue ;
        end
        
        hisNums(j) = sum(f_ref.pars.histCompNums(id)) ;
        
        
        fr_0.mu = f_ref.mu(:,id) ;
        fr_0.weights = f_ref.weights(id) ;
        fr_0.covariances = f_ref.covariances(:,:,id) ;
        
        fd_0.weights = sum(fr_0.weights) ;
        fd_0.mu = sum(fr_0.mu .* fr_0.weights)/sum(fr_0.weights) ;
        cc = reshape(fr_0.covariances,1,length(fr_0.weights)) ;
        fd_0.covariances = sum(fr_0.weights.*(cc + (fd_0.mu-fr_0.mu).^2))/sum(fr_0.weights) ;
        
        fd_0 = refitGaussianMixtureFunApproxSingle( fd_0, fr_0 ) ;
        if ( fd_0.weights == 0 ) valid(j) = 0 ; end
        
        
        f_dest.mu(:,j) = fd_0.mu ;
        f_dest.weights(j) = fd_0.weights ;
        f_dest.covariances(:,:,j) = fd_0.covariances ;
    end
    
id = find(valid==1) ;
f_dest.mu = f_dest.mu(:,id) ;
f_dest.weights = f_dest.weights(id) ;
f_dest.covariances = f_dest.covariances(:,:,id) ;
len_dest = length(id) ;

f_dest.weights = f_dest.weights / sum(f_dest.weights) ;
f_dest.covariances = reshape(f_dest.covariances,len_dest,d^2,1) ;

f_dest.pars.histCompNums = hisNums ;

% ------------------------------------------------------------- %
function [pair, H_min] = getMinimumDistancePair( H )

H_min = sum(sum(H)) ;
pair = [] ;
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
H = zeros(length(pdf_reduced.weights),length(pdf_reduced.weights)) ;
for i = 1 : length(pdf_reduced.weights)
    w1 = pdf_reduced.weights(i) ;
    mu1 = pdf_reduced.mu(:,i) ;
    C1 =  pdf_reduced.covariances(:,:,i) ;

    f1.weights = w1 ;
    f1.mu = mu1 ;
    f1.covariances = C1 ;

    for j =i+1 : length(pdf_reduced.weights)
        w2 = pdf_reduced.weights(j) ;
        mu2 = pdf_reduced.mu(:,j) ;
        C2 = pdf_reduced.covariances(:,:,j) ;
                H(i,j) = hellinger2Norm( w1, mu1, C1, w2, mu2, C2 ) ;
                
                
%         f2.weights =w2 ;
%         f2.mu = mu2 ;
%         f2.covariances = C2 ;
%         
%         pdf_n.weights = [f2.weights, f1.weights] ;
%         pdf_n.mu = [f2.mu, f1.mu] ;
%         pdf_n.covariances = cat(3,f2.covariances,f1.covariances) ;
%         H(i,j) = getHellError( pdf_n ) ;

        
        
%         H(i,j) = uHellingerJointSupport(  f1, f2 ) ;
    end
end

W = repmat(pdf_reduced.weights',1,length(pdf_reference.weights)) ; % reference weight matrix
N = (pdf_reference.weights)*pdf_reference.N ; % expected number of data-points

intMatrix.P = P.*W ; % P matrix
intMatrix.N = N ;
intMatrix.N_samples = pdf_reference.N ;
intMatrix.H = H ;
intMatrix.ownerships = {} ;
for i = 1:length(pdf_reduced.weights)
    intMatrix.ownerships = vertcat(intMatrix.ownerships, {i}) ;
end

% ------------------------------------------------------ %
function h = getHellingerAlltoOne( pdf_reference, w1, mu1, C1 )

% hellinger among pairs of components
h = zeros(length(pdf_reference.weights),1) ;
f1.weights = w1 ;
f1.mu = mu1 ;
f1.covariances = C1 ;

for j = 1 : length(pdf_reference.weights)
    w2 = pdf_reference.weights(j) ;
    mu2 = pdf_reference.mu(:,j) ;
    C2 = pdf_reference.covariances(:,:,j) ;
    h(j) = hellinger2Norm( w1, mu1, C1, w2, mu2, C2 ) ;


%     f2.weights = pdf_reference.weights(j) ;
%     f2.mu = pdf_reference.mu(:,j) ;
%     f2.covariances = pdf_reference.covariances(:,:,j) ;
%     pdf_n.weights = [f2.weights, f1.weights] ;
%     pdf_n.mu = [f2.mu, f1.mu] ;
%     pdf_n.covariances = cat(3,f2.covariances,f1.covariances) ;
%     h(j) = getHellError( pdf_n ) ;
    
    
    
    
%     h(j) = uHellingerJointSupport(  f1, f2 ) ;
end


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
         combinePairOfComponents( pdf_reference, pdf_reduced, intMatrix, pair  )

compress_this = 0 ;
% mask for components
s = ones(length(pdf_reduced.weights),1) ; 
s(pair) = 0 ;
S = repmat( s, 1, cols(intMatrix.P) ) ;
% mask generated

Cw = intMatrix.P ;
N = intMatrix.N ;

% expected log-likelihood for full distribution
Lf = sum(log2(sum(Cw,1)).*N) ;

% calculate the auxiliary component and integrals
ownerships_current = [intMatrix.ownerships{pair(1)},intMatrix.ownerships{pair(2)}] ;

id = ownerships_current ;
fr_0.mu = pdf_reference.mu(:,id) ;        
fr_0.weights = pdf_reference.weights(id) ;        
fr_0.covariances = pdf_reference.covariances(:,:,id) ;
                
[m0, C0] = momentMatching( fr_0.weights, fr_0.mu, fr_0.covariances ) ;

fd_0.weights = sum(fr_0.weights) ;        
fd_0.mu = m0 ;        
fd_0.covariances = C0 ;

fd_0 = refitGaussianMixtureFunApproxSingle( fd_0, fr_0 ) ;
alpha0 = sum(fd_0.weights) ;
% auxiliary component calculated

% make auxiliary vector
K = zeros(1,cols(Cw)) ;
for ( k = 1 : length(K) )
    m2 = pdf_reference.mu(:,k) ;
    C2 = pdf_reference.covariances(:,:,k) ;
    K(k) = normpdf(m0, m2, [], C0 + C2 ) ;
end

% expected log-likelihood for compressed distribution
Lc = sum(log2(sum(Cw.*S,1) + K*alpha0).*N) ;

% Minimum Description Lengths
mdl_f = getMDL( rows(pdf_reference.mu), length(pdf_reference.weights), Lf, intMatrix.N_samples ) ;
mdl_c = getMDL( rows(pdf_reduced.mu), length(pdf_reduced.weights)-1, Lc, intMatrix.N_samples ) ;
 
if ( mdl_c < mdl_f   )%&& abs(mdl_f - mdl_c)/abs(mdl_f) > 0.01
    compress_this = 1 ;
end

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
    pdf_reduced.mu = [mm, m0] ;
    pdf_reduced.covariances = cat(3,cc,C0) ;

    pdf_reduced.E = pdf_reduced.weights*pdf_reduced.N ;

    % update ownerships
    intMatrix.ownerships = vertcat(intMatrix.ownerships(id),{ownerships_current}) ;    
    
    % update Hellinger matrix
    intMatrix.H = intMatrix.H(id,id) ;
    h = getHellingerAlltoOne( pdf_reduced, alpha0, m0, C0 ) ;
    intMatrix.H = [[intMatrix.H, h(1:length(h)-1)]; h'] ;
end

% ----------------------------------------------------------------------- %
function f_dest = reEstimateDestination( f_ref, f_dest, ownerships )
len_refs = length(f_ref.weights) ;
len_dest = length(f_dest.weights) ;
d = rows(f_dest.mu) ;
f_ref.covariances = reshape(f_ref.covariances',d,d,len_refs) ;
f_dest.covariances = reshape(f_dest.covariances',d,d,len_dest) ;


hisNums = zeros(1,len_dest) ;
valid = ones(1,len_dest) ;
len_dest = length(f_dest.weights) ;
    for j = 1 : len_dest
        f0.weights = f_dest.weights ;
        
        
        id = find(ownerships==j) ;
        if isempty(id)
            f_dest.weight(j) = 0 ;
            continue ;
        end
        
        hisNums(j) = sum(f_ref.pars.histCompNums(id)) ;
        
        
        fr_0.mu = f_ref.mu(:,id) ;
        fr_0.weights = f_ref.weights(id) ;
        fr_0.covariances = f_ref.covariances(:,:,id) ;
        
        fd_0.weights = sum(fr_0.weights) ;
        fd_0.mu = sum(fr_0.mu .* fr_0.weights)/sum(fr_0.weights) ;
        cc = reshape(fr_0.covariances,1,length(fr_0.weights)) ;
        fd_0.covariances = sum(fr_0.weights.*(cc + (fd_0.mu-fr_0.mu).^2))/sum(fr_0.weights) ;
        
        fd_0 = refitGaussianMixtureFunApproxSingle( fd_0, fr_0 ) ;
        if ( fd_0.weights == 0 ) valid(j) = 0 ; end
        
        
        f_dest.mu(:,j) = fd_0.mu ;
        f_dest.weights(j) = fd_0.weights ;
        f_dest.covariances(:,:,j) = fd_0.covariances ;
    end
    
id = find(valid==1) ;
f_dest.mu = f_dest.mu(:,id) ;
f_dest.weights = f_dest.weights(id) ;
f_dest.covariances = f_dest.covariances(:,:,id) ;
len_dest = length(id) ;

f_dest.weights = f_dest.weights / sum(f_dest.weights) ;
f_dest.covariances = reshape(f_dest.covariances,len_dest,d^2,1) ;

f_dest.pars.histCompNums = hisNums ;

% ----------------------------------------------------------------------- %

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

w = w / sum(w) ;
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
