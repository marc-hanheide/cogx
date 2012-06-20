function demo_NEW_IKDE()
%close all


% TO DO:
% -----------------------------------
%   - fixed uncertainty derivation
%   - nonstationary update derivation
%   - generate samples from gauss, uni, gauss
%   - generate samples from heavily squed distribution
%   - generate sequence from a nonstationarry distribution
%   - *why do I increase the variances by an order of two? (some theoretical background)
%       + solution: Provided by Hellinger. But how to defend why do we even require inflation?    
%   - [Chi2 correction of the bandwidths (or simply start from enough
%      samples in experiments)]
%   - Start with a mixture of optimal and Silverman bandwidth with
%     annealing.
%   

% TO DO now:
%   -  implement Leonardis MDL in terms of Hellinger distance
%
%
%  SOLUTIONS:
%   - define inflation by increase of Hellinger distance to 0.1: 
%        Gives good inflation and consequently good compression. Hellinger
%        does not work for small distances!!!!
%
% Comparison:  
%   - Arandjelovic Cippola incremental algorithm
%   - VBSMS algorithm
%
%  FUTURE WORK:
%   - multidimensional derivations
%   - replace LM with partitioned L2 minimization
%   

global h_plugin hellErrorGlobal ;


export_to_movie = 0 ;
aviobj = initializeMovie( export_to_movie, 'C:\incremental_mix4.avi') ;
dat_scale = 1.5 ;



N=1000;
bounds = [-1 1 -1 1]*dat_scale ;
int = [bounds(1):(bounds(2)-bounds(1))/100:bounds(2)] ;
prior_adapt = 0.5 ;
nMaxComponents = 10 ;
nMaxComponentsPrior = 10 ;
C_sensor = 0* 0.005^2 ;%  1.1111e-005 ;%

%install the paths required for demonstration
installDemo1Paths() ;
 %newPath = 'C:\Program Files\MATLAB704\work\IncrementalKDE\optimal_bw_dll\' ; rmpath(newPath) ; addpath(newPath) ;
%data
xs=(rand(1,N)-1)*dat_scale;
 
% x1 = randn(1,floor(N/3))*0.1 - 0.3 ;
% x2 = randn(1,floor(N/3))*0.1 + 0.3 ;
% x3 = randn(1,floor(N/3))*1 ;
% xs = [x1, x2, x3] ;

% 
x1 = (rand(1,N*3/10)-1) ;
x2 = randn(1,floor(N*7/10))*0.1 + 0.5 ;
xs = [x1, x2] ;
% 
% 
% x1 = (rand(1,floor(N*2/10))*2-1) ;
% x2 = randn(1,floor(N*2/10))*0.01 + 0.7 ;
% x3 = (rand(1,floor(N*2/10))-1)*0.25 ;
% x4 = randn(1,floor(N*3/10))*0.02 -0.5 ;
% xs = [x1, x2, x3,x4] ;
% 
% 


% hellErrorGlobal = 0.08 ;
% hellErrorGlobal = -1.5 ;

% fname = ['uniform','.txt'] ; % gaussMixture uniGauss uniform difMix difMix2
% % % save(fname,'xs','-ascii') ;
% xs = load(fname)  ;

% N = 1000 ;
[xs, pdf] = generateValuesPdf( N, 'MaWa', 3, 'MyDi', 0)  ; %1, 'LoadData', 'uniNormX2', 'LoadData', 'uniNormStrange'  , 'LoadData', 'xUniNorm', 'LoadData', 'X1',  , 'LoadData', 'MaWaX', 'LoadData', 'XX', 'LoadData', 'MaWaX' 'LoadData', 'MaWaX' , , 'PermuteData', 1, 'SaveData', 'MaWaX' , 'LoadData', 'MaWaX' ,'LoadData', 'MaWaX'

% cd 'D:\MATLAB704\work\NewKDE_code\GaMMoT\demo\ExperimentsAndEvaluations'
% [data, L] = loadExperimentalData( 'MaWa_type3' )
%  xs = data.X{27} ;

shift = mean(xs) ;
 
xs = (xs - shift) ;
scale = 1/(max(xs)-min(xs))*1.8 ; 
xs = xs *scale;

pdf.norm = rescaleDistribution( pdf.norm, scale, shift, 'forward' ) ;

% xs = sort(xs) ;

N = length(xs) ;
% I = randperm(N) ;
% xs = xs(I) ;

%   fname = 'difUniHard.txt' ; save(fname,'xs','-ascii') ;

%xs(1) = -0.5 ; xs(2) = +0.5 ;  xs(3) = -0.5 ;  xs(4) = +0.5 ;  xs(5) = -0.5 ;

D = abs(xs(1) - xs(2))
figure(5); m = sprintf('%f',D) ; title(m)
%xs(3) = xs(2) ;
% xs(4) = xs(5) ;


%xs = [-1:1/N:-1/N] ;
ys=(rand(1,N)*2-1)*dat_scale;
X=[xs;ys];


%figure(3); hist(xs,20);

%figure
figure(1);clf

subplot(2,2,1);
title('Input data');
line([0,0],[-1,1]*dat_scale,'color','g');
xlabel('x');ylabel('y');
axis([-1,1.5,-1,1]*dat_scale);
hold on;

subplot(2,2,2);
title('Single Gaussian');
line([0,0],[-1,1]*dat_scale,'color','g');

subplot(2,2,3);
title('1-D Kernel density - incremental');
line([0,0],[-1,1]*dat_scale,'color','g');


subplot(2,2,4); 
[hst, hst_x] = hist(xs,10) ; hst = hst / sum(hst) ; bar(hst_x,hst) ; axis tight ; ca=axis;
line([0,0],[0,ca(4)],'color','g');
axis([[-1,1]*dat_scale,0,ca(4)]);
title('1-D Kernel density - batch');

%init single Gaussian
mu = X(1,1) ; sg = [0] ; n = 1 ;
[mu,sg,n] = updateMV(X(1,2),mu,sg,n);
% initialize kernel
C0 = getSilvermanBWfromGaussian( sg, n ) ; % *0.5^2

% C0 = slow_univariate_bandwidth_estimate_STEPI(2,X(1,1:2)) ;

% example of how to get optimally estimated pdf
% pdf = get_1d_OptimalKDE( X(1,1:2) ) ;
% % showDecomposedPdf( pdf ) ;
% C0 = pdf.covariances(1) ;
C0 = C0 + C_sensor ;


% initialize the KDE
initializationMethod = 'Plugin' ; % 'Silverman'
reportProgress = 0 ;
scaleErrorThreshold = 1/0.7 ; 1/0.7 ;1.5 ;
hellErrorGlobal = 0.1 ;% 0.11 / scaleErrorThreshold ;
nMaxComponents = 10 ;
nMaxComponentsPrior = nMaxComponents ;
N_observed = nMaxComponents ;
kernel_pdf_X = updateIKDE( [], X(1,1:N_observed), ...
                        'initialize', 1 ,'nMaxComponentsPrior', nMaxComponentsPrior,...
                        'scaleErrorThreshold', scaleErrorThreshold,...
                        'hellErrorGlobal', hellErrorGlobal,...
                        'initializationMethod', initializationMethod);
% KDE initialized                      
                        
                        
kernel_pdf =  kernel_pdf_X ;              
                        
% nMaxComponents = 10 ;
% nMaxComponentsPrior = nMaxComponents ;
% N_observed = nMaxComponents ;
% kernel_pdf = initializeKDE( X(1,1:N_observed), N_observed ) ;
% mu = kernel_pdf.scale.mu ; 
% sg = kernel_pdf.scale.sg ;
% n = N_observed ;
% kernel_x0 = X(1,1) ;
% kernel_pdf.H0 = -1 ;                        
 
% nMaxComponents = 2 ;
% N_observed = 2 ;
% kernel_pdf.mu = X(1,1:2) ;
% kernel_pdf.weights = [1 1] ; 
% kernel_pdf.weights = kernel_pdf.weights / sum(kernel_pdf.weights) ;
% kernel_pdf.covariances = [1; 1]*C0 ;
% kernel_pdf.components = 2 ;
% kernel_x0 = mu ; % initial point used for initialization if required
% kernel_pdf.H0 = -1 ;
% kernel_pdf.H0 = C0 ;
% kernel_pdf.pars.Wt1 = 1 ; 
% kernel_pdf.pars.wt1 = 0.5 ;
% kernel_pdf.pars.histCompNums = [1 1] ;
% kernel_pdf.scale.mu = mu ; 
% kernel_pdf.scale.sg = sg ;
% % kernel_pdf.N = n ;
% kernel_pdf.nMaxComponents = nMaxComponents ;

kernel_kde = kernel_pdf ;

mdl_pdf = kernel_pdf ;

subplot(2,2,1);
scatter (X(1,1),X(2,1),'b.');

H = [] ;
NN = [[1,2];[1,2]] ;
global N_eff ;

h_fast = 1
s1 = 1 ;
%incremental
for i = N_observed+1 : N
    aviobj = recordImage( export_to_movie, aviobj ) ;

    x=X(1,i);
    [mu,sg,n] = updateMV(x',mu,sg,n);

    kpdf_prev = kernel_pdf ;

    % h_fast = slow_univariate_bandwidth_estimate_STEPI(i,X(1,1:i)) ;
    %  h_plugin = h_fast ;

    %    prior_adapt = 0.4*(exp(-i/10))+0.5
    % update current KDE
    
%     kernel_kde = updateIKDE( kernel_kde, x ) ;
    
    if ( i == 103 )
        dfg = 56 ;
    end

    kernel_x0 = [] ;
    hncp = kernel_pdf.pars.histCompNums ;
%     kernel_pdf = updateKDE( kernel_pdf, kernel_x0, x, sg, n, prior_adapt, 1, C_sensor, mu ) ;
 
    % Update KDE
    kernel_pdf_X = updateIKDE( kernel_pdf_X, x, 'reportProgress', 0 ) ;              
    % Updated
    
    kernel_pdf = kernel_pdf_X ;                    
                        

kernel_pdf.pars.histCompNums = [hncp, 1] ;

    NN = [NN,[N_eff;kernel_pdf.components]] ;

    H = [H, sqrt([h_fast^2;h_plugin.^2])] ;

    % compress distribution if the number of components exceeds "nMaxComponents"
%     if ( length(kernel_pdf.weights) > nMaxComponents )
%         H0 = kernel_pdf.H0 ;
%         components = kernel_pdf.components ;
%         pars = kernel_pdf.pars ;
%         tic
% 
% 
%         %         C0 = getSilvermanBWfromGaussian( sg, n )*0.1 ;
% 
%         %         C0 = min(kernel_pdf.covariances) * 0.5 ;
%         C0 = [] ;
% 
%                 kernel_pdf = compressDistribution( kernel_pdf, 'showIntermediate', 0, ...
%                               'inflationVariance', C0, 'hellErrorGlobal', ...
%                               hellErrorGlobal, 'scaleErrorThreshold', scaleErrorThreshold,...
%                               'reportProgress', reportProgress ) ;
%                 kernel_pdf.pars.histCompNums = [] ;
% 
% %         kernel_pdf = selfOrganizeDistribution_test( kernel_pdf, 1 ) ;
% 
% 
%         kernel_pdf.N = i ;
% %         kernel_pdf = selfOrganizeDistributionMDL( kernel_pdf, kernel_pdf ) ;
% 
% %         kernel_pdf = selfOrganizeDistribution( kernel_pdf, kernel_pdf ) ;
% %                   kernel_pdf = compressSubpart( kernel_pdf ) ;
% 
% 
% %         [mdl_pdf ] = getMDLoptimalDistribution( kernel_pdf, kernel_pdf ) ;
% %         kernel_pdf = mdl_pdf ;
%         
% %         hncp = kernel_pdf.pars.histCompNums ;
% %         figure(2) ; clf
% %         subplot(1,2,1) ; showdecomposedpdf(kernel_pdf) ; title(sprintf('Original distribution, Noc: %d', length(kernel_pdf.weights))) ; a = axis ;
% %         subplot(1,2,2) ; showdecomposedpdf(mdl_pdf) ; title(sprintf('MDL compressed distribution, Noc: %d', length(mdl_pdf.weights))) ;  axis(a) ;
% %         drawnow;
% 
% 
%         % hell = suHellinger( kernel_pdf, kpdf_prev ) ;
%         % msg = sprintf('Hellinger: %f',hell) ;
% %         figure(1); subplot(2,2,1) ; title(msg)
% 
%         if ( length(kernel_pdf.weights) > nMaxComponents  )
%             nMaxComponents = 1.5 * nMaxComponents ;
%         elseif ( length(kernel_pdf.weights)< nMaxComponents/1.5 )
%             nMaxComponents =  nMaxComponents/1.5 ;
%         end
%         
% 
%         %        if ( nMaxComponents < nMaxComponentsPrior ) nMaxComponents = nMaxComponentsPrior ; end
% 
% 
%         
%         toc
%         kernel_pdf.components = components ;
%         kernel_pdf.pars = pars ;
%         kernel_pdf.H0 = H0 ;
%         kernel_pdf.pars.histCompNums = hncp ;
%     end
 
   figure(1)                              
   subplot(2,2,1); hold off ;
   title('Input data');
   line([0,0],[-1,1]*dat_scale,'color','g');
   xlabel('x');ylabel('y');
   axis([-1,1.5,-1,1]*dat_scale) ; hold on ;
   scatter(X(1,1:i),X(2,1:i),'b.') ;

   % draw Gaussian
   subplot(2,2,2);
   y=normpdf(int,mu(1),sqrt(sg(1)));
   plot(int,y);
   ca=axis;
   line([0,0],[0,ca(4)],'color','g');
   axis([[-1,1]*dat_scale,0,ca(4)]);
   title('Single Gaussian');

   % draw Kernel-based estimate
   subplot(2,2,3) ; hold off ; plot(0,0,'.'); 

%   figure(2); clf
%    showPdfEstimateResults( [], [], kernel_pdf.mu, kernel_pdf.weights, kernel_pdf.covariances, bounds, [], [] ) ;
%    showPdfEstimateResults( [], [], kernel_kde.mu, kernel_kde.weights, kernel_kde.covariances, bounds, [], [] ) ;
    showDecomposedUniNormMixPdf(pdf, 'decompose', 0, 'bounds', [-1.2,1], 'linTypeSum', 'b') ; hold on ;
%     showDecomposedPdf(kernel_pdf, 'decompose', 0, 'bounds', [-1.2,1], 'linTypeSum', 'r') ;
    showDecomposedPdf(kernel_pdf_X, 'decompose', 0, 'bounds', [-1.2,1], 'linTypeSum', 'g--') ;
    


   ca=axis;
   line([0,0],[0,ca(4)],'color','g');
   axis([[-1,1]*dat_scale,0,ca(4)]);
   msg = sprintf('1-D incremental KDE,\n N_{components}=%d', length(kernel_pdf.weights)) ; 
   title(msg);

   subplot(2,2,4); 
   [hst, hst_x] = hist(xs(1:i),10) ; hst = hst / sum(hst) ; bar(hst_x,hst) ; axis tight ; ca=axis;
   line([0,0],[0,ca(4)],'color','g');
   axis([[-1,1]*dat_scale,0,ca(4)]);
   msg = sprintf('1-D histogram on %d observed data', i) ; title(msg)
   drawnow;
   

 %  figure(1); title(sprintf('Ncomp pdf(ref): %d, Ncomp kde(novi): %d', length(kernel_pdf.weights), length(kernel_kde.weights))) ;
   
%    figure(3) ; clf ; hold on ;
%    plot([1:length(H(1,:))],H(1,:),'r') ;
%    plot([1:length(H(2,:))],H(2,:),'g') ;
%    if rows(H)> 2 plot([1:length(H(3,:))],H(3,:),'b') ; end
%    drawnow ;
   
%    figure(4); clf ; hold on ;
%    plot( NN(1,:) ,NN(2,:),'r') ;  axis equal;
   
   
    
%    F = [20,120,500] ;
%    if ( sum(i==F)> 0 ) 
%        showCurrentResult(kernel_pdf, xs(1:i), 2);
%        pause ; 
%    end
   
end 
aviobj = closeMovie( aviobj, export_to_movie ) ;
 



function e = compressSubpart( e )

minMembers = 2 ;
scaleCovariances = 1/1.5^2 ;
plot_int_res = 0 ;
plot_final_res = 0 ;
[new_centers, new_weights, new_covariances] = ...
         approximateDensity2( e.mu, e.weights, e.covariances, ... 
                              plot_int_res , plot_final_res,...
                              minMembers, scaleCovariances ) ;
e.mu = new_centers ;
e.weights = new_weights ;
e.covariances = new_covariances ;


% ----------------------------------------------- % 
function kernel_pdf = initializeKDE( data, nMaxComponents ) 
 
mu = data ; sg = cov(data) ;
% C = getSilvermanBWfromGaussian( sg, length(data) ) ;
pdf = get_1d_OptimalKDE( data ) ; 
C = pdf.covariances(1) ; 
covariances = ones(length(data),1) * C ;
kernel_pdf.mu = data ;
kernel_pdf.weights = ones(1, length(data))/sum(ones(1, length(data))) ;  
kernel_pdf.covariances = covariances ;
kernel_pdf.components = length(data) ;
kernel_pdf.pars.Wt1 = 1 ; 
kernel_pdf.pars.wt1 = 0.5 ;
kernel_pdf.pars.histCompNums = [1 1] ;
 
kernel_pdf.scale.mu = mu ; 
kernel_pdf.scale.sg = sg ;
kernel_pdf.nMaxComponents = nMaxComponents ;
kernel_pdf.Curr_covariances = covariances(:)' ;

% ----------------------------------------------- % 


% ----------------------------------------------------------------------- %
function showCurrentResult(f_ref, data, fignum)
figure(fignum); clf; 
dat_scale = 1 ;

b1 = sqrt(max([f_ref.covariances])) ;
bmin = min([f_ref.mu]) - b1*5 ;
bmax = max([f_ref.mu]) + b1*5 ;
bounds = [bmin,bmax] ;

subplot(1,3,1) ; hold on ;
[hst, hst_x] = hist(data,[-1:0.1:0]) ; hst = hst / sum(hst) ; bar(hst_x,hst) ; axis tight ; ca=axis;
line([0,0],[0,ca(4)],'color','g');
axis([[-1,1]*dat_scale,0,ca(4)]);

subplot(1,3,2) ; 
pdf_pos = constructKDEfromData( data, 'compression', 0 ) ;
for i = 1 : length(pdf_pos.weights)
   showPdf( bounds, 1000, pdf_pos.mu(:,i), pdf_pos.covariances(i,:), pdf_pos.weights(i), 'k', 1) ; 
end
showPdf( bounds, 1000, pdf_pos.mu, pdf_pos.covariances, pdf_pos.weights, 'r',2  ) ;
ca = axis ;
line([0,0],[0,ca(4)],'color','g');   
axis([[-1,1]*dat_scale,0,ca(4)]);

subplot(1,3,3) ; hold on ;
for i = 1 : length(f_ref.weights)
   showPdf( bounds, 1000, f_ref.mu(:,i), f_ref.covariances(i,:), f_ref.weights(i), 'k',1 ) ; 
end 
showPdf( bounds, 1000, f_ref.mu, f_ref.covariances, f_ref.weights, 'r',2  ) ;
ca = axis ;
line([0,0],[0,ca(4)],'color','g');   
axis([[-1,1]*dat_scale,0,ca(4)]);
 
drawnow ;
% ----------------------------------------------------------------------- %
function y_evals = showPdf( bounds, N,centers, covariances, weights, color, lw )
x_evals = [bounds(1):abs(diff(bounds))/N:bounds(2)] ;
y_evals = evaluateDistributionAt( centers, weights, covariances, x_evals ) ;
plot ( x_evals, y_evals, color, 'LineWidth',lw )



% ---------------------------------------------------------- %
% ---------------------------------------------------------- %
function aviobj = initializeMovie( export_to_movie, aviFileName ) 
aviobj = [] ;
if ( export_to_movie == 1 )
    framesPerSec = 10;
    aviobj = avifile(aviFileName,'fps',framesPerSec');
end
% ------------------------ %
function aviobj = recordImage( export_to_movie, aviobj )
if ( export_to_movie == 1 )
    h = gcf; % get current figure handle
    set(h,'Color',[1 1 1]); % set background color to black
    aviobj = addframe(aviobj,h);
end
% ------------------------ %
function aviobj = closeMovie( aviobj, export_to_movie ) 
if ( export_to_movie == 1 )
     aviobj = close(aviobj) ; 
else
    aviobj = [] ;
end
% ------------------------ %
function installDemo1Paths() 
% install Kernel learning
newPath = '..\incrementalPDF\' ; rmpath(newPath) ; addpath(newPath) ;
newPath = '..\evaluation\' ; rmpath(newPath) ; addpath(newPath) ;

newPath = 'D:\Work\Matlab\IncrementalKDE\GaMMoT\tcimm\' ; rmpath(newPath) ; addpath(newPath) ;


installIncrementalKDEUpdatePaths() ;
 
initial_path = pwd ;
cd .. ; 
installGaMMotPaths() ;
cd( initial_path ) ;
