function demonstrations()
% 
%
% Demonstrations:
%   demo 1: learns a distribution from sequence of data, then it unlearns
%           a part of distribution from some new data points
%   demo 2: estimates likelihood of some data and demonstrates
%           likelihood estimation using a marginal from oKDE
%   demo 3: demonstrates estimation of a spiral distribution
%   demo 4: demonstrates estimation of a nonstationary distribution
%   demo 5: demonstrates estimation of a distribution by weighting
%           samples w.r.t. their typicality.
%
% Author: Matej Kristan, 2010.

% installEntireMaggot3() ; 
 
% disp('----- Demonstating 1D learning and unlearning example ---------') ;
% demonstrateLearninigUnlearning1D() ;
% 
% disp('----- Demonstating likelihood evaluation and marginalization ---------') ;
% demoEvaluations() ;
% 
% disp('----- Demonstating 2D estimation ---------') ;
% demo2Destimation() ;

% disp('----- Demonstrating 2d sub est') ;
demo2D1Destimation() ;
%  
% disp('------ Demonstrating nonstationary distribution --------');
% estimateNonstat() ;
% 
% disp('----- Demonstating weighted 1D learning example ---------') ;
% demo1Destimation() ;
% 
% disp('----- Draw a rabbit -------------------------------------') ;
% drawaRabbit() ;

% ----------------------------------------------------------------------- %
function drawaRabbit()
    
rbt = load('D:\Work\Matlab\IncrementalKDE\Maggot2\demo\tests\pictures\rbt.mat') ;  
rbt = rbt.rbt ;

N = size(rbt,1) ;
Dth =   0.02 ; % allowed Hellinger distance error in compression
N_init = 2 ;
switchSelectionSeeds = 1 ;
modd = 5 ;
dat = rbt' ;

I = randperm(N) ; dat = dat(:,I) ;

kde = executeOperatorIKDE( [], 'input_data', dat(:,1:N_init),'add_input' );
kde = executeOperatorIKDE( kde, 'compressionClusterThresh', Dth ) ;
kde = executeOperatorIKDE( kde, 'switchSelectionSeeds', switchSelectionSeeds ) ;
for i = N_init+1 : size(dat,2) 
    tic
    kde = executeOperatorIKDE( kde, 'input_data', dat(:,i), 'add_input'  ) ;
    toc
    if mod(i,modd)==0 
        figure(1) ; clf ;
        hold on ;
        plot(dat(1,1:i), dat(2,1:i), '.', 'MarkerEdgeColor', [1, 0.4, 0.4]) ;
        executeOperatorIKDE( kde, 'showKDE','showkdecolor', 'b' ) ;
        msg = sprintf('Samples: %d ,Comps: %d \n', i , length(kde.pdf.w) ) ; 
        title(msg) ; axis equal ;  
    end
    
    
end
% ----------------------------------------------------------------------- %
function estimateNonstat()

N_max = 8000 ;
N_effLim = 1000 ;
morphfact = 1 - 1/N_effLim ;
MaW1 =  3 ; MyDi1 = 0 ;
MaW2 = 0*6 ; MyDi2 = 9+0*6 ;    

% generate intial samples from first mixture
[x_init, pdf] = generateValuesPdf( 3, 'MaWa',MaW1, 'MyDi', MyDi1); 
[x1, pdf1] = generateValuesPdf( 50000,'MaWa',MaW1, 'MyDi', MyDi1); 
[x2, pdf2] = generateValuesPdf( 50000,'MaWa',MaW2, 'MyDi', MyDi2); 
 

% initialize the oKDE
kde = executeOperatorIKDE( [], 'input_data', x_init,'add_input','kde_w_attenuation', morphfact  ) ; 

w_mix1 = 1 ;
for i = 1 : N_max 
    if i > 1000
       w_mix1 = w_mix1*morphfact ;         
    end
    
    % get pdf and the sampled datapoint
    [x, pdf_ref] = generateNonstatDistributionsAndVals(MaW1, MaW2, MyDi1, MyDi2, x1, x2, w_mix1, i) ;
    
    % update oKDE
    tic 
    kde = executeOperatorIKDE( kde, 'input_data', x,'add_input'  ) ;
    toc
    
    % plot results
    figure(1) ; clf ; 
    showDecomposedUniNormMixPdf(pdf_ref, 'decompose', 0, 'linTypeSum','--g' ) ;
    hold on ;
    executeOperatorIKDE( kde, 'showKDE','showkdecolor', 'b' ) ;
    plot(x, x*0, 'o', 'MarkerSize',10, 'MarkerEdgeColor','b', 'MarkerFaceColor','w', 'LineWidth',2 ) ;  
    a = axis ; axis([-3.5, 3,0, a(4)]) ;
    msg = sprintf('Observed %d, Number of components %d, N_{eff}=%1.3g, w_{mix1}=%1.3g', i, length(kde.pdf.w), kde.ikdeParams.N_eff, w_mix1 ) ;
    title(msg) ;
    
end
% ----------------------------------------------------------------------- %
function demo2Destimation()

% generate some datapoints 
N = 10000 ;
Dth = 0.05 ; 0.02 ; % allowed Hellinger distance error in compression
N_init = 10 ;
modd = 5 ;
dat = generateSpiral(N,[]) ;

kde = executeOperatorIKDE( [], 'input_data', dat(:,1:N_init),'add_input' );
kde = executeOperatorIKDE( kde, 'compressionClusterThresh', Dth ) ;
for i = N_init+1 : size(dat,2) 
    tic
    kde = executeOperatorIKDE( kde, 'input_data', dat(:,i), 'add_input'  ) ;
    toc
    if mod(i,modd)==0 
        figure(1) ; clf ;
        hold on ;
        plot(dat(1,1:i), dat(2,1:i), '.', 'MarkerEdgeColor', [1, 0.4, 0.4]) ;
        executeOperatorIKDE( kde, 'showKDE','showkdecolor', 'b' ) ;
        msg = sprintf('Samples: %d ,Comps: %d \n', i , length(kde.pdf.w) ) ; 
        title(msg) ; axis equal ; axis([-13, 13, -13, 13])
    end
    
    
end

% ----------------------------------------------------------------------- %
function demo2D1Destimation()

% generate some datapoints 
selectSubDimensions = 2 ;
th_start_true = 300 ;
N = 10000 ;
Dth = 0.05 ; 0.02 ; % allowed Hellinger distance error in compression
N_init = 10 ;
modd = 5 ;
dat = generateSpiral(N,[]) ;

kde = executeOperatorIKDE( [], 'input_data', dat(:,1:N_init),'add_input' );
kde = executeOperatorIKDE( kde, 'compressionClusterThresh', Dth ) ;
for i = N_init+1 : size(dat,2) 
    if th_start_true < i
        selectSubDimensions = [] ;
    end
    
    tic
    kde = executeOperatorIKDE( kde, 'input_data', dat(:,i), 'add_input', 'selectSubDimensions', selectSubDimensions  ) ;
    toc
    if mod(i,modd)==0 
        figure(1) ; clf ;
        hold on ;
        plot(dat(1,1:i), dat(2,1:i), '.', 'MarkerEdgeColor', [1, 0.4, 0.4]) ;
        executeOperatorIKDE( kde, 'showKDE','showkdecolor', 'b' ) ;
        msg = sprintf('Samples: %d ,Comps: %d \n', i , length(kde.pdf.w) ) ; 
        title(msg) ; axis equal ; axis([-13, 13, -13, 13])
    end
    
    
end


% ----------------------------------------------------------------------- %
function demoEvaluations()

disp('Estimate oKDE from 30 samples') ;
Nl = 30 ; 
dat = rand(3,Nl) ;
kde = executeOperatorIKDE( [], 'input_data', dat,'add_input' ) ;

x = rand(3,5) ;
disp('Evaluatate likelihood of  some points under the oKDE') ;
selectSubDimensions = [] ;
res = executeOperatorIKDE( kde, 'input_data', x, 'evalPdfOnData', 'selectSubDimensions', selectSubDimensions ) ;
res.subRegularized
res.evalpdf

disp('Evaluatate likelihood of some points under the oKDE by ignoring the second dimension') ;
selectSubDimensions = [1 3] ;
res = executeOperatorIKDE( kde, 'input_data', x, 'evalPdfOnData', 'selectSubDimensions', selectSubDimensions ) ;
res.subRegularized
res.evalpdf

disp('Evaluatate typicality of some points under the oKDE by ignoring the second dimension') ;
disp('Typicality is defined as the ratio between the probability of a point under the pdf and') ;
disp('the probability of the most probable point (maximum of pdf).') ;
selectSubDimensions = [1 3 ] ;
res = executeOperatorIKDE( kde, 'input_data', x, 'evalTypOnData', 'selectSubDimensions', selectSubDimensions ) ;
res.subRegularized
res.evaltyp
kde = res.kde ;

% ----------------------------------------------------------------------- %
function demonstrateLearninigUnlearning1D()

switchSelectionSeeds = 0 ;    
    
% generate data for the model
N = 100 ; N_init = 5 ;
[x, pdf] = generateValuesPdf( N, 'MaWa',0, 'MyDi',6 ) ; 
x_init = x(:,1:N_init) ;
x = x(:,N_init+1:length(x)) ;

% initialize the KDE
kde = executeOperatorIKDE( [], 'input_data', x_init, 'add_input', 'switchSelectionSeeds', switchSelectionSeeds ) ;
t1 = 0 ;
% incrementally add data and show result
for i = 1 : size(x,2)
    % add a data point
    tic
    kde = executeOperatorIKDE( kde, 'input_data', x(:,i), 'add_input' ) ; 
    t1 = t1 + toc ;
 
    % draw reference, model and estimate
    figure(1); clf ; subplot(1,2,1) ;
    showDecomposedUniNormMixPdf(pdf, 'decompose', 0, 'linTypeSum', '--g') ; hold on ; 
    executeOperatorIKDE( kde, 'showKDE', 'showkdecolor', 'b' ) ;
    plot(x(1:i),zeros(1,i),'r*') ;
end
msg = sprintf('Average time per adition: %1.5g s', t1/size(x,2)) ; disp( msg ) ;

% select negative points for unlearning
x_neg = rand(1,20)*1 + 0.5 ;
plot(x_neg, x_neg*0, 'b+') ; plot(x_neg, x_neg*0, 'mO') ;
title('Learning/unlearning using points only')


% unlearn with points
tic
kde_unlwpts = executeOperatorIKDE( kde, 'input_data', x_neg,'unlearn_with_input' ) ;
t1 = toc ; msg = sprintf('Time for unlearning using samples: %1.5g s', t1 ) ; disp( msg ) ;

% show result
showDecomposedUniNormMixPdf(pdf, 'decompose', 0, 'linTypeSum', '--g') ; hold on ; 
drawDistributionGMM( 'pdf', kde.pdf, 'decompose', 0, 'color', 'b' ) ;
drawDistributionGMM( 'pdf', kde_unlwpts.pdf, 'decompose', 0, 'color', 'r' ) ;
plot(x_neg, x_neg*0, 'mO') ;

% end

% Example 2: unlearning with another kde
% initialize the negative KDE
 kde_neg = executeOperatorIKDE( [], 'input_data', x_neg, 'add_input' ) ;
% unlearn with kde
tic ;
kde_unlwkde = executeOperatorIKDE( kde, 'input_data', kde_neg, 'unlearn_with_input' ) ;
t1 = toc ; msg = sprintf('Time for unlearning using negative kde: %1.5g s', t1 ) ; disp( msg ) ; 

figure(2) ; clf ;
showDecomposedUniNormMixPdf(pdf, 'decompose', 0, 'linTypeSum', '--g') ; hold on ; 
drawDistributionGMM( 'pdf', kde.pdf, 'decompose', 0, 'color', 'b' ) ;
drawDistributionGMM( 'pdf', kde_neg.pdf, 'decompose', 0, 'color', 'c' ) ;
drawDistributionGMM( 'pdf', kde_unlwkde.pdf, 'decompose', 0, 'color', 'r' ) ;
title('Unlearning using kdes')
 
figure(1) ;
% incrementally add data and show result
for i = 1 : size(x,2)
    % add a data point
    tic
    kde_unlwpts = executeOperatorIKDE( kde_unlwpts, 'input_data', x(:,i), 'add_input' ) ; 
    t1 = t1 + toc ;
 
    % draw reference, model and estimate
    subplot(1,2,2) ; hold off ;
    showDecomposedUniNormMixPdf(pdf, 'decompose', 0, 'linTypeSum', '--g') ; hold on ; 
    executeOperatorIKDE( kde_unlwpts, 'showKDE', 'showkdecolor', 'b' ) ; drawnow ;
    plot(x(1:i),zeros(1,i),'r*') ;
end
title('Unlearnt KDE with additional learning')
 
% ----------------------------------------------------------------------- %
function demo1Destimation()
% Contents:
% Samples are generated from a distribution and noise is added to the
% samples. Each sample is weighted w.r.t. its typicality. This demonstrates
% how to use different a priori weights for each input sample.
    
% generate some datapoints 
N = 1000 ;
Dth = 0.02 ; % allowed Hellinger distance error in compression
modd = 5 ;
sensitivity = 0.9 ; % ~ 0.5 will focus on dominant modes; ~2 will weight everything approximately the same

MaW1 = 0 ; 
MyDi1 = 5 ;
[dat, pdf_ref] = generateValuesPdf( N,'MaWa',MaW1, 'MyDi', MyDi1) ;
sig = 0.3 ; % standard deviation of the additive noise

kde = executeOperatorIKDE( [], 'input_data', dat(:,1:3),'add_input' );
kde = executeOperatorIKDE( kde, 'compressionClusterThresh', Dth ) ;
for i = 4 : size(dat,2) 
    
    % add noise to the data-point 
    dat(:,i) = dat(:,i) + randn()*sig ;
    
    % evaluate typicality of data-point under current KDE
    res = executeOperatorIKDE( kde, 'input_data', dat(:,i), 'evalTypOnData'  ) ;
    
    % set the data-point weigth to its typicality
    obs_relative_weights = exp(-0.5*((1-res.evaltyp)/sensitivity).^2)  ;
    tic
    kde = executeOperatorIKDE( kde, 'input_data', dat(:,i), 'add_input', 'obs_relative_weights', obs_relative_weights  ) ;
    toc
    if mod(i,modd)==0 
        figure(1) ; clf ;
        hold on ;
        showDecomposedUniNormMixPdf(pdf_ref, 'decompose', 0, 'linTypeSum','--g' ) ;
        plot(dat(1:i), 0*dat(1:i), '.', 'MarkerEdgeColor', [1, 0.4, 0.4]) ;
        executeOperatorIKDE( kde, 'showKDE','showkdecolor', 'b' ) ;
        plot(dat(i), dat(i)*0, 'o', 'MarkerSize',10, 'MarkerEdgeColor','b', 'MarkerFaceColor','w', 'LineWidth',2 ) ;         
        msg = sprintf('Samples: %d ,Comps: %d \n, Distortion: %1.3g', i , length(kde.pdf.w),sig ) ; 
        title(msg) ; 
    end
end


% ----------------------------------------------------------------------- %
 function x = generateSpiral(N,t0)

if nargin < 3
    genIm = 0 ;
end

a = 1 ;
b = 1 ;
% t0 = [] ;
if nargin < 2 || isempty(t0) %||  length(t0) < 2
    theta = rand(1,N)*10 ; %25 ;
else
    theta = 10*t0 ;
end

sig = 0.9 ; %1 ;%0.1 ;
r = a + b*theta ;
y = sin(theta).*r + randn(1,N)*sig ; 
x = cos(theta).*r + randn(1,N)*sig ;
x = [x;y] ;


% ----------------------------------------------------------------------- %
function [x, pdf_ref] = generateNonstatDistributionsAndVals(MaW1, MaW2, MyDi1, MyDi2, x1, x2, w_mix1, i)         
    [x0, pdf_ref1] = generateValuesPdf( 1, 'MaWa',MaW1, 'MyDi', MyDi1) ;
    [x0, pdf_ref2] = generateValuesPdf( 1, 'MaWa',MaW2, 'MyDi', MyDi2) ;
    if ~isfield(pdf_ref2,'uni')
        pdf_ref2.uni.mu=[] ;pdf_ref2.uni.widths=[] ; pdf_ref2.uni.weights=[] ;
    end
    if ~isfield(pdf_ref1,'uni')
        pdf_ref1.uni.mu=[] ;pdf_ref1.uni.widths=[] ; pdf_ref1.uni.weights=[] ;
    end
    
    pdf_ref.uni.mu = [pdf_ref1.uni.mu, pdf_ref2.uni.mu] ;
    pdf_ref.uni.widths = [pdf_ref1.uni.widths, pdf_ref2.uni.widths] ;
    pdf_ref.uni.weights = [pdf_ref1.uni.weights*w_mix1, pdf_ref2.uni.weights*(1-w_mix1)] ;
    pdf_ref.norm.mu = [pdf_ref1.norm.mu, pdf_ref2.norm.mu] ;
    pdf_ref.norm.weights = [pdf_ref1.norm.weights*w_mix1, pdf_ref2.norm.weights*(1-w_mix1)] ;
    pdf_ref.norm.covariances = vertcat(pdf_ref1.norm.covariances, pdf_ref2.norm.covariances ) ;
    
    
    cs = cumsum([w_mix1 (1-w_mix1)]) ;
    if rand(1) < cs(1)
         x = x1(i) ;
    else
         x = x2(i) ;
    end
    
    
 