function gradient() %( f1, f2 )
 
newPath = 'C:/Program Files/MATLAB704/work/IncrementalKDE/' ; rmpath(newPath) ; addpath(newPath) ;
newPath = 'C:\Program Files\MATLAB704\work\IncrementalKDE\optimalIncrementalKDE\' ; rmpath(newPath) ; addpath(newPath) ;
newPath = 'C:\Program Files\MATLAB704\work\IncrementalKDE\unlearning\' ; rmpath(newPath) ; addpath(newPath) ;
newPath = 'c:\Program Files\MATLAB704\work\IncrementalKDE\vbwms\' ; rmpath(newPath) ; addpath(newPath) ;


f1_mix = getMixture( 2 ) ;
f2_mix = getMixture( 3 ) ;
showme(f1_mix, f2_mix) ;

disp('Generating derivatives...')
% generate derivatives of cost function for incremental approximation
[E, f2]= generateCostFunction(f1_mix, f2_mix) ;
derivatives = getDerivatives( E, f2 ) ;
% derivatives generated

alpha = 1 ;
disp('Optimization in progress...') ;
for i = 1 : 10
    f2 = updateParamters( derivatives, f2, alpha ) ;
    showme(f1_mix, f2.f) ;
end


function f2 = updateParamters( derivatives, f2, alpha )
D = evaluateDerivativesAt( derivatives, f2 ) ;

t0 = 1 ; t1 = t0 + f2.f.siz - 1 ;
f2.f.covariances = f2.f.covariances - D(t0:t1) ;
f2.f.covariances = abs(f2.f.covariances) ;
t0 = t1 + 1 ; t1 = t0 + f2.f.siz - 1 ;
f2.f.weights = f2.f.weights - D(t0:t1) ;
t0 = t1 + 1 ; t1 = t0 + f2.f.siz - 1 ;
f2.f.means = f2.f.means - D(t0:t1) ;
f2.f.weights = abs(f2.f.weights) ;
f2.f.weights = f2.f.weights / sum(f2.f.weights) ;
    
    

% ----------------------------------------------------------------------- %
function showme(f1_mix, f2_mix)

figure(1); clf ; hold on ; bounds = [-4, 15] ;
showPdf( bounds, 100, f1_mix.means, f1_mix.covariances', f1_mix.weights, 'g' ) ;
showPdf( bounds, 100, f2_mix.means, f2_mix.covariances', f2_mix.weights, 'r' ) ;
drawnow ;

function y_evals = showPdf( bounds, N,centers, covariances, weights, color )
x_evals = [bounds(1):abs(diff(bounds))/N:bounds(2)] ;
y_evals = evaluateDistributionAt( centers, weights, covariances, x_evals ) ;
norm = sum(y_evals) ;
sign_p = sum(y_evals) < 0 ; 
y_evals = y_evals / norm * (-1)^(sign_p) ;    

plot ( x_evals, y_evals, color )

% ----------------------------------------------------------------------- %
function D = evaluateDerivativesAt( derivatives, f2 )

D = [] ;
len = length(derivatives) ;
for i = 1 : len
    msg = sprintf('Evaluating derivative: %d', i); disp(msg) ;
    d = replaceValues( derivatives{i}, f2 ) ; 
    d = double(d) ;
    D = [D, d] ;     
end



% ---------------------------------------------------------------- %

% ---------------------------------------------------------------- %
function derivatives = getDerivatives( E, f2 )

derivatives = {} ;
% derivatives over covariances
for i = 1 : f2.f.siz
    c_curr = f2.covars{i} ;    
    dE = diff(E, c_curr) ;
    derivatives = horzcat(derivatives, {dE}) ;
end

% derivatives over weights
for i = 1 : f2.f.siz
    w_curr = f2.weights{i} ;
    dE = 0; %diff(E, w_curr) ;
    derivatives = horzcat(derivatives, {dE}) ;
end
    
% derivatives over means
for i = 1 : f2.f.siz
    w_curr = f2.means{i} ;
    dE = diff(E, w_curr) ;
    derivatives = horzcat(derivatives, {dE}) ;
end

% ---------------------------------------------------------------- %
function [E, f2]= generateCostFunction(f1_mix, f2_mix)

syms x ;
f1 = constructMixture( f1_mix, '1_' ) ;
f2 = constructMixture( f2_mix, '2_' ) ;

Fm = replaceValues( f1.Fm, f1 ) ;
f1.Fm = simplify(collect(Fm)) ;

e0 = - 2*collect(f2.Fm*f1.Fm) + collect(f2.Fm*f2.Fm) ;
E = int(e0,x, -inf, inf) ;


% ---------------------------------------------------------------- %
function Fm = replaceValues( Fm, F ) 

for i = 1 : F.f.siz
    c_curr = F.covars{i} ;
    m_curr = F.means{i} ;
    w_curr = F.weights{i} ;  
    Fm = subs(Fm, m_curr, F.f.means(i)) ;
    Fm = subs(Fm, c_curr, F.f.covariances(i)) ;
    Fm = subs(Fm, w_curr, F.f.weights(i)) ;
end

% ---------------------------------------------------------------- %
function f = getMixture( siz )

f.siz = siz ;
f.means = rand(1,siz)*10 ;
f.weights = rand(1,siz) ;
f.weights = f.weights / sum(f.weights) ;
f.covariances = abs(rand(1,siz)) ;

% ---------------------------------------------------------------- %
function f_out = constructMixture( f, mfix )

Fm = 0 ;
syms m C ;
N = makeGauss(m, C) ;
len = f.siz ;
covars = {} ;
means = {} ;
weights = {} ;
for i = 1 : len
    c_curr = sprintf('C%s%d',mfix,i) ;
    m_curr = sprintf('m%s%d',mfix,i) ;
    w_curr = sprintf('w%s%d',mfix,i) ;
    N_curr = subs(N,C, c_curr) ;
    N_curr = subs(N_curr,m, m_curr) ;
    means = horzcat(means, {m_curr}) ;
    covars = horzcat(covars, {c_curr}) ;
    weights = horzcat(weights, {w_curr}) ;
    Fm = Fm + w_curr*N_curr ;
end

f_out.covars = covars ;
f_out.means = means ;
f_out.weights = weights ;
f_out.Fm = Fm ;
f_out.f = f ;

% ---------------------------------------------------------------- %
function N = makeGauss(m, C)

syms m1 C1 x ;
sq2 = sym('sqrt(2*pi)')  ;
N = 1/( sq2*sqrt(C1)) * exp(-0.5*(m1-x)^2/C1) ;
N = subs(N,C1, C) ;
N = subs(N,m1, m) ;
N = simplify(N) ;
 