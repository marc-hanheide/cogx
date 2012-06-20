function f0 = reducedSetCompression( f1, varargin )
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

global fignum inflationVariance ;
fignum = 1 ;

disp(' ') ;
disp('-------------------------------------------------------------')
disp('Compressing distributions...') ;
disp('-------------------------------------------------------------'); 
disp(' ') ;
maxItFitting = 20 ;         % maximum cycles of fitt-remove iterations
maxLMFitIterations = 15 ;   % maximum number of fine-fitting iterations
in_maxLMFitIterations = 5 ; % maximal number of intermediate fitting iterations
showIntermediate = 0 ;      % toggle visualizations
gradient = 1 ;              % use LevenbergMarquadt with gradient descent (1) or least squares (2)
finely_refit = 1 ;          % perform multiple iterations of fine-fitting at the end

inflationVariance = [] ;

pruning = 'SMO' ;

% calculate scale of the input distribution
[scale shift]= getScaleOfDistribution( f1 ) ;
% rescale the distribution
f1 = rescaleDistribution( f1, scale, shift, 'forward' ) ;

f0 = f1 ;
args = varargin;
nargs = length(args);
for i=1:2:nargs
    switch args{i}
        case 'modify', f0 = createInitialRBFset( f1, 'modify', args{i+1} )  ;
        case 'f_init', f0 = args{i+1}; f0 = rescaleDistribution( f0, scale, shift, 'forward' ) ;
        case 'showIntermediate', showIntermediate = args{i+1};
        case 'gradient', gradient = args{i+1};
        case 'pruning', pruning = args{i+1};
        case 'finely_refit', finely_refit = args{i+1};
        case 'inflationVariance', inflationVariance = args{i+1}*scale.^2; 
    end
end

counter = 0 ;
init_num_modes = cols(f0.mu) ;
init_num_modes0 = init_num_modes ;
sw = 0 ;
 
% remove some units
[f0,alpha] = removeUnits( pruning, f0, f0, f_ref_data_X, f_ref_data_Y ) ;
for i = 1 : maxItFitting
    counter = counter + 1 ;
    
    % --- begin compression step --- 
    % refit parameters 
    [scale_var, hl ]= getOptimalScaleToRef( f0, f1 ) ;
    
  % figure(3);  clf; plot(f1.weights); hold on ; plot(f0.weights,'--r')    
    % remove components weights
    disp('Removing components...') ;
    [f0,alpha] = removeUnits( pruning, f0, f0, f_ref_data_X, f_ref_data_Y ) ;
    % --- end compression step --- 

    if ( showIntermediate == 1 )
        showme(f0) ;
        figure(2); clf; plot(alpha) ; title('Recomputed weights. Zero weights will be removed') ;
    end

    curr_num_modes = length(f0.weights) ;
    reportInt(init_num_modes0, init_num_modes, curr_num_modes) ;
    if ( init_num_modes == curr_num_modes) 
        break ;
       if ( sw == 1 ) break ; else sw = 1 ; end
    else
        sw = 0 ;
    end
        
    init_num_modes = curr_num_modes ;
end
f0.weights = f0.weights / sum(f0.weights) ;


% inflate distribution if required
[scale_var, hl ]= getOptimalScaleToRef( f0, f1 ) ;
figure(5); title(sprintf('Final inflation: %f, Remaining error: %f',scale_var, hl)); drawnow ;
f0.covariances = f0.covariances*scale_var ;


% rescale the distribution
f0 = rescaleDistribution( f0, scale, shift, 'backward' ) ;




% report results
reportFin(maxItFitting, counter, init_num_modes0, curr_num_modes) ;

% ----------------------------------------------------------------------- %
function [f0,alpha]= removeUnits( pruning, f0, f1, f_ref_data_X, f_ref_data_Y )
global inflationVariance ;

 
scale_var = 2; 2 ; 4 ; % 1.5^2 ;


if ( isempty(inflationVariance) )
    HellError = 0.1 ;
    [scale_var, hl ]= getOptimalScale( f0, HellError ) ;
    figure(5); title(sprintf('Self-inflation: %f at predefined Hellinger: %f, calculated: %f',scale_var, HellError, hl)); drawnow ;
    fx = f0 ;
    fx.covariances = fx.covariances*scale_var ;
else
    fx = f0 ;
    fx.covariances = fx.covariances + inflationVariance ;
    
end

if isequal(pruning,'SMO')
%    fx = f0 ;
%    fx.covariances = fx.covariances*scale_var ; 
 
% hell = suHellinger( f0, fx ) ;
% msg = sprintf('Hellinger: %f',hell) ; figure(5); title(msg) ; 

   
   
      alpha = optimizeWeights( f1, fx ) ;   
  %    alpha = optimizeWeights( f0, fx ) ;       
   f0 = pruneMixture( f0, alpha ) ;
%   f0.covariances = fx.covariances(find(alpha>0)) ; 
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