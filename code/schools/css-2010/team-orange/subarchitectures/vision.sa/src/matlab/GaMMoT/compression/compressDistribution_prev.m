function f0 = compressDistribution( f1, varargin )
%
% Matej Kristan (2007)
%
% ------------------------------------
% Compresses a mixture of Gaussians.
% ------------------------------------
% Input:
% f1   ... Input distribution. Structure:
%           f1.mu          ... row vector of mean values
%           f1.covariances ... column vector of covariances
%           f1.weights     ... row vector of weights
% varargin:
%   'showIntermediate' ...  Toggle visualization (0,1).

% Output:
% f0   ... Output distribution. Structure same as f1. 
%

global fignum inflationVariance hellErrorGlobal ;
fignum = 1 ;


maxItFitting = 20 ;         % maximum cycles of fitt-remove iterations
maxLMFitIterations =  15 ;   % maximum number of fine-fitting iterations
in_maxLMFitIterations = 5;10; 5 ; % maximal number of intermediate fitting iterations
showIntermediate = 0 ;      % toggle visualizations
gradient = 1 ;              % use LevenbergMarquadt with gradient descent (1) or least squares (2)
finely_refit = 1 ;          % perform multiple iterations of fine-fitting at the end
hellErrorGlobal = 0.1 ;
neg_fit = 0 ;
inflationVariance = [] ;
scaleErrorThreshold = inf ; % 1.5
reportProgress = 1 ;

pruning = 'SMO' ;


f1 = resortComponents( f1, 'ascend' ) ;

% calculate scale of the input distribution
[scale shift]= getScaleOfDistribution( f1 ) ;
% rescale the distribution
f1 = rescaleDistribution( f1, scale, shift, 'forward' ) ;

f0 = f1 ;
args = varargin;
nargs = length(args);
for i=1:2:nargs
    switch args{i}
        case 'modify',
            f0 = createInitialRBFset( f1, 'modify', args{i+1} ) ; neg_fit = 1 ;
        case 'f_init', f0 = args{i+1}; f0 = rescaleDistribution( f0, scale, shift, 'forward' ) ;
        case 'showIntermediate', showIntermediate = args{i+1};
        case 'gradient', gradient = args{i+1};
        case 'pruning', pruning = args{i+1};
        case 'finely_refit', finely_refit = args{i+1};
        case 'inflationVariance', inflationVariance = args{i+1}*scale.^2; 
        case 'hellErrorGlobal', hellErrorGlobal = args{i+1} ;
        case 'scaleErrorThreshold', scaleErrorThreshold = args{i+1} ;
        case 'reportProgress', reportProgress = args{i+1} ;
    end
end


if reportProgress == 1
    disp(' ') ;
    disp('-------------------------------------------------------------')
    disp('Compressing distributions...') ;
    disp('-------------------------------------------------------------');
    disp(' ') ;
end

counter = 0 ;
init_num_modes = cols(f0.mu) ;
init_num_modes0 = init_num_modes ;
sw = 0 ;

if ~isfield(f1, 'components')
    f1.components = [] ;
end
components = f1.components ;

% generate points for compression
[f_ref_data_X, numSigPoints ]= getPointsOnDistribution( f1, 5, 6 ) ;
[f_ref_data_Y] = evaluateDistributionAt( f1.mu, f1.weights, f1.covariances, f_ref_data_X ) ;

hellErrorGlobal_reference = hellErrorGlobal ;

curr_num_modes = length(f0.weights) ;
f0_lastGood = f1 ;
% remove some units
% [f0,alpha] = removeUnits( pruning, f0, f0, f_ref_data_X, f_ref_data_Y ) ;

for i = 1 : maxItFitting    
    counter = counter + 1 ;

    
    % --- begin compression step --- 
    % refit parameters 
%     tic
 % remove components weights
    f0 = resortComponents( f0,  'descend' ) ;
    if ( reportProgress == 1 )
        disp('Removing components...') ;
    end
    [f0,alpha] = removeUnits( pruning, f0, f1, f_ref_data_X, f_ref_data_Y, reportProgress ) ;

    
    H = uHellingerJointSupport( f0, f1 ) 
    
    f0 = resortComponents( f0,  'descend' ) ;
    % showIntermediate = 1 ;
    if ( reportProgress == 1 )
        disp('Fitting parameters...') ;
    end
    f0 = refitGaussianMixture( f1, varargin{:},...
        'showIntermediate', showIntermediate, ...
        'opt_weights', 1, 'maxIterations', in_maxLMFitIterations, 'f_init', f0, 'gradient', gradient ) ;
%     toc
      f0.components = components ;
 
%    f0 = UEM( f1, f0, 'maxIterations' , 15 ) ;

  % figure(3);  clf; plot(f1.weights); hold on ; plot(f0.weights,'--r')    
   
    if ( showIntermediate == 1 )
        showme(f0) ;
        figure(2); clf; plot(alpha) ; title('Recomputed weights. Zero weights will be removed') ;
    end

    if ( neg_fit == 0 && hellErrorGlobal > 0 )
        H = uHellingerJointSupport( f0, f1 ) ;
        if ( H > hellErrorGlobal*scaleErrorThreshold )
            f0 = f0_lastGood ;
            break ;
%             hellErrorGlobal = hellErrorGlobal *0.5 ;
%         else
%             hellErrorGlobal = hellErrorGlobal_reference ;
        end
    end
 
    curr_num_modes = length(f0.weights) ;
    
    if reportProgress == 1
        reportInt(init_num_modes0, init_num_modes, curr_num_modes) ;
    end
    if ( init_num_modes == curr_num_modes) 
        break ;
       if ( sw == 1 ) break ; else sw = 1 ; end
    else
        sw = 0 ;
    end
        
    init_num_modes = curr_num_modes ;
    f0_lastGood = f0 ;
end

 

f0 = pruneMixture( f0, f0.weights' ) ;
f0.weights = f0.weights / sum(f0.weights) ;

hellErrorGlobal = hellErrorGlobal_reference ;

f0 = resortComponents( f0,  'descend' ) ;

% % remove components weights     
if ( finely_refit == 1 ) 
    
%     
%    f0 = refitGaussianMixture( f1, varargin{:},...
%         'showIntermediate', showIntermediate, ...
%         'opt_weights',1, 'maxIterations', 2*in_maxLMFitIterations, 'f_init', f0, 'gradient', gradient ) ;
%  
%     
    
    
    
%       maxItFitting = 80 ;
  
   gradient = 0 ; maxItFitting = 10 ; %3
   %    gradient = 1 ; maxItFitting = 600 ;
   if ( reportProgress == 1 )
       disp('Fine-fitting parameters...') ; %   gradient = 1 ;
   end
    f0 = refitGaussianMixture( f1, varargin{:},...
        'showIntermediate', showIntermediate, ...
        'opt_weights',1, 'maxIterations', maxItFitting, 'f_init', f0, 'gradient', gradient ) ;
    %     gradient = 1 ;
    %   maxItFitting = 80
    %     f0 = refitGaussianMixture( f1, varargin{:},...
    %         'showIntermediate', showIntermediate, ...
    %         'opt_weights',1, 'maxIterations', maxItFitting, 'f_init', f0, 'gradient', gradient ) ;
    %     [f0,alpha] = removeUnits( 'SMO', f0, f1, f_ref_data_X, f_ref_data_Y ) ;
    %     curr_num_modes = length(f0.weights) ;
    if ( reportProgress == 1 )
        disp('Finished!')
    end
end
f0.weights = f0.weights / sum(f0.weights) ;


% inflate distribution if required
% [scale_var, hl ]= getOptimalScaleToRef( f0, f1 ) ;
% figure(5); title(sprintf('Final inflation: %f, Remaining error: %f',scale_var, hl)); drawnow ;
% f0.covariances = f0.covariances*scale_var ;


% rescale the distribution
f0 = rescaleDistribution( f0, scale, shift, 'backward' ) ;
 
% report results
if ( reportProgress == 1 )
    reportFin(maxItFitting, counter, init_num_modes0, curr_num_modes) ;
end

% ----------------------------------------------------------------------- %
function f1 = resortComponents( f1, mode ) 
% sort components by ascending weigths

y = evaluateDistributionAt( f1.mu, f1.weights, f1.covariances, f1.mu ) ;
[a, id ]=sort(y,mode) ; 
% [a, id ]=sort(f1.weights,mode) ; 

f1.weights = f1.weights(id) ;
f1.mu = f1.mu(:,id) ;
f1.covariances = f1.covariances(id,:) ;


% ----------------------------------------------------------------------- %
function [f0,alpha]= removeUnits( pruning, f0, f1, f_ref_data_X, f_ref_data_Y, reportProgress )
% f0 ... target
% f1 ... reference

global inflationVariance hellErrorGlobal ;

f0.weights = abs(f0.weights) ;
f0.weights = f0.weights / sum(f0.weights) ;

% [f0, alpha] = removeUnitsRSDE( pruning, f0, f1 ) ; return 
% 
%  
scale_var = 2; 2 ; 4 ; % 1.5^2 ;


if ( isempty(inflationVariance) )
%     HellError = 0.1 ;
%     [scale_var, hl ]= getOptimalScale( f0, HellError ) ;
%     figure(5); title(sprintf('Self-inflation: %f at predefined Hellinger: %f, calculated: %f',scale_var, HellError, hl)); drawnow ;
%     fx = f0 ;
%     fx.covariances = fx.covariances*scale_var ;


if hellErrorGlobal >= 0
    maxInflation = 10 ;
    HellError = hellErrorGlobal ; %0.2 ; 0.1; 0.05; %0.05/sqrt(2) ;
%     [scale_var, hl ]= getOptimalScale( f0, HellError ) ;
    [scale_var, hl ]= getOptimalScale2( f0, f1, HellError, maxInflation ) ;
    
    if ( reportProgress == 1 )
        hl = hl - hellErrorGlobal ;
        H = uHellingerJointSupport( f0, f1 ) ;
        figure(5); title(sprintf('Self-inflation: %1.3g at predefined Hellinger: %1.3g, Error in calculation: %1.3g, FinalDistance: %1.3g',scale_var, HellError, hl, H)); drawnow ;
    end
else
    scale_var = -hellErrorGlobal ;
    if ( reportProgress == 1 )
        figure(5); title(sprintf('Self-inflation predefined: %1.3g',scale_var )); drawnow ;
    end
end
    
    fx = f0 ;
    fx.covariances = f0.covariances * scale_var ;

%     fx = f0 ;
%     scale_var = 1.7 ;
%     
%     fx.covariances = f0.covariances * scale_var ;
% 
%     [likRat, likRat2] = getAvLikRatioAtSigmaPoints( f0, fx ) ;
%      figure(5); title(sprintf('Self-inflation: %1.3g at calculated ALR: %1.3g, %1.5g',scale_var, likRat*100, likRat2)); drawnow ;

else
%     fx = f0 ;
%     fx.covariances = fx.covariances + inflationVariance ;
%     
end

if isequal(pruning,'SMO')
%    fx = f0 ;
%    fx.covariances = fx.covariances*scale_var ; 
 
% hell = suHellinger( f0, fx ) ;
% msg = sprintf('Hellinger: %f',hell) ; figure(5); title(msg) ; 

%   f0.covariances = f0.covariances * scale_var ;
      alpha = optimizeWeights( f1, fx ) ;   
  %    alpha = optimizeWeights( f0, fx ) ;       
   f0 = pruneMixture( f0, alpha ) ;
   
%    f1 = pruneMixture( f1, alpha ) ;
%    f0.weights = f1.weights ; f0.weights = f0.weights / sum(f0.weights) ;
%    f0.covariances = f0.covariances*(1 + (scale_var-1)*0.2) ; %fx.covariances(find(alpha>0)) ; 
elseif isequal(pruning,'tabu')
  %  showPdfNow(f0,f_ref_data_X, f_ref_data_Y) ;
    alpha = optimizeWeights_MDL( f_ref_data_X, f_ref_data_Y, f0, f1 ) ;
    f0 = pruneMixture( f0, alpha ) ;
else
    error(['Unknown pruning method: ',pruning]) ;
end
% ----------------------------------------------------------------------- %
function showPdfNow(f1_mix, X, Y)
global showInterm fignum;

%if showInterm ~= 1 return ; end
figure(fignum); hold on ; 

b1 = sqrt(max([f1_mix.covariances])) ;
bmin = min([f1_mix.mu]) - b1*5 ;
bmax = max([f1_mix.mu]) + b1*5 ;
bounds = [bmin,bmax] ;
%bounds = [-4, 40] ;
showPdf( bounds, 100, f1_mix.mu, f1_mix.covariances, f1_mix.weights, '--k' ) ;
plot(X, Y, 'ok') ; title('Current optimization result.')
drawnow ;

% ----------------------------------------------------------------------- %
function reportInt(init_num_modes0, init_num_modes, curr_num_modes)

disp(' ') ;
msg = sprintf('Components removed: %d', init_num_modes-curr_num_modes ) ; 
disp(msg) ;
msg = sprintf('Current number of components: %d (of initial %d)', curr_num_modes, init_num_modes0) ; 
disp(msg) ;

% ----------------------------------------------------------------------- %
function reportFin(maxItFitting, counter, init_num_modes0, curr_num_modes)

disp(' ')
disp('-----  ') ;
msg = sprintf('Compression finished in %d iterations.', counter ) ; 
msg = sprintf('Number of components remaining: %d (of initial %d)', curr_num_modes, init_num_modes0) ; 
disp(msg) ; disp('-----  ') ;

% ----------------------------------------------------------------------- %
function f = pruneMixture( f, alpha )

id = find(alpha>0) ;
f.weights = alpha(id)'; 
% f.weights = f.weights(id)/sum(f.weights(id)) ;
f.mu = f.mu(:,id) ;
f.covariances = f.covariances(id,:) ;


% ----------------------------------------------------------------------- %
function showme(f1_mix)
global showInterm fignum;

if showInterm ~= 1 return ; end
figure(fignum);  hold on ;
b1 = sqrt(max([f1_mix.covariances])) ;
bmin = min([f1_mix.mu]) - b1*5 ;
bmax = max([f1_mix.mu]) + b1*5 ;
bounds = [bmin,bmax] ;
showPdf( bounds, 100, f1_mix.mu, f1_mix.covariances, f1_mix.weights, 'b' ) ;
drawnow ;

% ----------------------------------------------------------------------- %
function y_evals = showPdf( bounds, N,centers, covariances, weights, color )
x_evals = [bounds(1):abs(diff(bounds))/N:bounds(2)] ;
y_evals = evaluateDistributionAt( centers, weights, covariances, x_evals ) ;
plot ( x_evals, y_evals, color )