function g = UEM( f, g, varargin )
% Matej Kristan (2007)
%
% f ... reference distribution
% g ... initial approximative distribution
%
%
global showInterm fignum ;
fignum = 1 ;

k = 0 ;
minTolerance = 1e-6 ;
maxIterations = 50 ;
args = varargin;
nargs = length(args);
for i=1:2:nargs
    switch args{i}
        case 'maxIterations', maxIterations = args{i+1} ;
        case 'minTolerance', minTolerance = args{i+1} ;
        case 'k_sigma', k = args{i+1} ;
        otherwise, ;
    end
end

k = 0 ;
% generate a set of sigma points from f
[X, sigPointsPerComponent ] = getAllSigmaPoints( f, k ) ;
pdf = evaluateDistributionAt( f.mu, f.weights, f.covariances, X ) ;

for i = 1 : maxIterations
    g = uEM_step( f, g, X, sigPointsPerComponent ) ;
    
     
 clf
    show_pdfs(f, g) ;
    plot(X,pdf,'+g') ;
 pause(0.3);
end



% ----------------------------------------------------------------------- %
function g = uEM_step( f, g, X, sigPointsPerComponent )
d = rows(f.mu) ;
len_g = length(g.weights) ;
len_f = length(f.weights) ;
len_X = length(X) ;

% Expectation
% generate the array of weights 
W = zeros(length(g.weights), length(X)) ;
for j = 1 : len_g
    pdf = evaluateDistributionAt( g.mu(:,j), g.weights(j), g.covariances(j,:), X ) ;
    W(j,:) = pdf ;
end
 
for i = 1 : len_X
    W(:,i) = W(:,i)/sum(W(:,i)) ;
end

% Maximization
% estimate the parameters

% get partial weight matrix
aW = zeros(len_g, len_X) ;
for i = 1 : len_f
    index = (i-1)*sigPointsPerComponent ;
    select = [index+1 : index + sigPointsPerComponent] ;
    aW(:,select) = W(:,select)*f.weights(i) ; 
end

% partial sum
pW = sum(aW,2)  ;

% weights
g.weights = pW';%/sigPointsPerComponent ;
g.weights = g.weights/sum(g.weights) ;

% mean values and covariance matrices
Mu_tmp = g.mu ;
d_sq = d*d ;
for j = 1 : len_g
    a = repmat(aW(j,:),d,1) ;
    Mu_tmp(:,j) = sum((a.*X)/pW(j),2) ;
    Mu = Mu_tmp(:,j);%g.mu(:,j) ;
    
    C = zeros(d,d) ;
    for i = 1 : len_X
        D = X(:,i) - Mu ;
        C = C + aW(j,i)*D*D' ;
    end
    g.covariances(j,:) = reshape(C,1,d_sq)/pW(j) ;
end
g.mu = Mu_tmp ; 
g = rationalize_pdf( g, W ) ;
length(g.weights)

% ----------------------------------------------------------------------- %
function covariances = rectifyCovariances( covariances )

for i = 1 : rows(covariances)
    if covariances(i,:) < 0.001 
        covariances(i,:) = 0.001 ;
    end
end
% ----------------------------------------------------------------------- %
function g = rationalize_pdf( g, W )

Th_w = 1e-6 ;
g.covariances = rectifyCovariances( g.covariances ) ;
a = ones(1,length(g.weights)) ;
for i = 1 :length(g.weights)
    if g.weights(i) < Th_w 
        a(i) = 0 ;
    end
    
    W(i,:) = W(i,:)/sum(W(i,:)) ;
end
% select = find(a>0) ;
% g.mu = g.mu(:,select) ;
% g.covariances = g.covariances(select,:) ;
% g.weights = g.weights(select) ;
% g.weights = g.weights / sum(g.weights) ;

T = size(W,1)*0.01 ;
C = sum(W>0.9,2) ;
select = find(C<1) ; 
if ( ~isempty(find(C>0)) )
    select = select(1) ;
    g.mu = g.mu(:, select) ;
    g.covariances = g.covariances(select,:) ;
    g.weights = g.weights(select) ;
    g.weights = g.weights/sum(g.weights) ;
end
% ----------------------------------------------------------------------- %
function show_pdfs(f,g)
global showInterm fignum;

if showInterm ~= 1 return ; end
figure(fignum);  hold on ;
b1 = sqrt(max([f.covariances])) ;
bmin = min([f.mu]) - b1*5 ;
bmax = max([f.mu]) + b1*5 ;
bounds = [bmin,bmax] ;
showPdf( bounds, 100, f.mu, f.covariances, f.weights, 'b' ) ;
showPdf( bounds, 100, g.mu, g.covariances, g.weights, 'r--' ) ;
title('Blue - reference, Red - result');
drawnow ;

% ----------------------------------------------------------------------- %
function y_evals = showPdf( bounds, N,centers, covariances, weights, color )
x_evals = [bounds(1):abs(diff(bounds))/N:bounds(2)] ;
y_evals = evaluateDistributionAt( centers, weights, covariances, x_evals ) ;
plot ( x_evals, y_evals, color )

