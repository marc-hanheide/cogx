function demo_UEM()

% install path
newPath = '..\..\uHellinger' ; rmpath(newPath) ; addpath(newPath) ;
newPath = '..\' ; rmpath(newPath) ; addpath(newPath) ;
newPath = '..\..\aux_tools' ; rmpath(newPath) ; addpath(newPath) ;


% X = zeros(600,1);
% X(1:200,:) = normrnd(0,1,200,1);
% X(201:400,:) = normrnd(0,2,200,1);
% X(401:600,:) = normrnd(0,3,200,1);
% X = normrnd(0,1,20,1);
% X = [ X;normrnd(0,2,20,1)]; 
% X = [X;normrnd(0,3,30,1)];
% 
% Init.W = [1,1,1]/3 ;
% Init.M = [0.5 1.5 2];
% Init.V = ones(1,1,3)*0.2^2 ;
% [W,M,V,L] = EM_GM(X,3,[],[],1,Init) 
% 
% % 
% X = normrnd(0,1,10,1);
% X = [ X;normrnd(0,2,20,1)]; 
% X = [X;normrnd(0,3,30,1)];
% [W,M,V,L] = EM_GM_fast(X,1,[],[],1,[]) 


N = 20 ;
f = generateGaussianMixture( 50, 1 ) ;
g = generateGaussianMixture( 2, 1 ) ;
 
UEM( f, f )



