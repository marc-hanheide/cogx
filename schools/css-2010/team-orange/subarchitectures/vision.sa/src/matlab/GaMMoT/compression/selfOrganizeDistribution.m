function f_dest = selfOrganizeDistribution( f_ref, f_dest )

maxIterations = 100 ;
specializedScales = [] ;

% figure(1); clf; debugShowX( f_ref, f_ref ) ;drawnow ;
tic
% remove units

if ( isempty(f_dest) ) f_dest = f_ref ; end 

r = rand(1,length(f_ref.weights))-0.5 ;
dm = reshape(f_ref.covariances,1,length(f_ref.weights))*0.1.*r ;
f_ref.mu = f_ref.mu + dm ;

% generate points for compression
% [f_ref_data_X, numSigPoints ]= getPointsOnDistribution( f_ref, 5, 6 ) ;
% [f_ref_data_Y] = evaluateDistributionAt( f_ref.mu, f_ref.weights, f_ref.covariances, f_ref_data_X ) ;



 remove_scale_factor = 2 ;
 
% specializedScales = getSpecializedScales( f_dest.pars.histCompNums ) ;
disp('removing units')
% remove units
f_dest = removeUnits( f_dest, f_ref, remove_scale_factor, specializedScales ) ;


% f_dest = removeUnits0 ( 'tabu', f_dest, f_ref, f_ref_data_X, f_ref_data_Y ) ;
% f_dest = reoptimizeWeights( f_dest, f_ref) ;

disp('calculating ownership')
    % find ownerships
    ownerships = getOwnerships( f_ref, f_dest ) ;

    % what would happen if we recalculated the new components via
    % weighted ownerships?
    
    disp('reestimating parameters')
    % reestimate parameters
    f_dest = reEstimateParameters( f_ref, f_dest, ownerships ) ;

    
    
      
numComps = length(f_ref.weights) ;
for i_org = 1 : maxIterations
    
%     specializedScales = getSpecializedScales( f_dest.pars.histCompNums ) ;
    disp('removing units')
    % remove units
    f_dest = removeUnits( f_dest, f_ref, remove_scale_factor, specializedScales ) ;
%     f_dest = removeUnits0 ( 'tabu', f_dest, f_ref, f_ref_data_X, f_ref_data_Y ) ;
    %   debugShowX( f_ref, f_dest ) ; drawnow ;

    disp('calculating ownership')
    % find ownerships
    ownerships = getOwnerships( f_ref, f_dest ) ;

    % what would happen if we recalculated the new components via
    % weighted ownerships?

    disp('reestimating parameters')
    % reestimate parameters
    f_dest = reEstimateParameters( f_ref, f_dest, ownerships ) ;


    numComps0 = length(f_dest.weights) ;

    disp(['Components removed: ',num2str(length(f_ref.weights) - numComps0)])
    
    if ( numComps0 == numComps )
        break ;
    else
        numComps = numComps0 ;
    end
end
% reestimate weights
% f_dest = reoptimizeWeights( f_dest, f_ref) ;

% toc
% figure(1); debugShowX( f_ref, f_dest ) ; drawnow ;
 
% --------------------------------------------------------------------- %
function specializedScales = getSpecializedScales( N )

I = find(N==1) ; N(I) = 2 ;
specializedScales = 2 ; (N./chi2inv(0.05,N-1)) ; %; 1./(N./chi2inv(0.95,N-1)) ;

% --------------------------------------------------------------------- %
function debugShowX( f_ref, f_dest )

clf ;
returnBounds = showDecomposedPdf( f_ref, 'linTypeSum', 'b', 'linTypeSub', 'b--' ) ;
showDecomposedPdf( f_dest, 'bounds', returnBounds ) ;


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

% ----------------------------------------------------------------------- %
function f0 = reoptimizeWeights( f0, f_ref)
alpha = optimizeWeights( f_ref, f0 ) ;
f0.weights = alpha' ;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%555
function [f0,alpha]= removeUnits0( pruning, f0, f_ref, f_ref_data_X, f_ref_data_Y )
% f0 ... target
% f1 ... reference

global inflationVariance ;
pruning = 'tabu' ;
 
scale_var = 2; 2 ; 4 ; % 1.5^2 ;

fx = f0 ;

%    HellError = 0.1; 0.05; 
%     [scale_var, hl ]= getOptimalScale2( f0, f_ref, HellError ) ;%f_ref

    scale_var =1 
    
% figure(5); title(sprintf('Self-inflation: %f at predefined Hellinger %f',scale_var, HellError)); drawnow ;


    fx.covariances = fx.covariances*scale_var ;

if isequal(pruning,'SMO')
%    fx = f0 ;
%    fx.covariances = fx.covariances*scale_var ; 
 
% hell = suHellinger( f0, fx ) ;
% msg = sprintf('Hellinger: %f',hell) ; figure(5); title(msg) ; 

   
   
      alpha = optimizeWeights( f1, fx ) ;   
  %    alpha = optimizeWeights( f0, fx ) ;       
   f0 = pruneMixture( f0, alpha ) ;
%    f0.covariances = f0.covariances*(1 + (scale_var-1)*0.2) ; %fx.covariances(find(alpha>0)) ; 
elseif isequal(pruning,'tabu')
  %  showPdfNow(f0,f_ref_data_X, f_ref_data_Y) ;
    alpha = optimizeWeights_MDL( f_ref_data_X, f_ref_data_Y,f0, f_ref  ) ;
    f0 = pruneMixture( f0, alpha ) ;
else
    error(['Unknown pruning method: ',pruning]) ;
end

% ----------------------------------------------------------------------- %
function f0 = removeUnits( f0, f_ref, scale_var, specializedScales )
%scale_var = 2 ; 4 ; % 1.5^2 ;

[f0, alpha] = removeUnitsRSDE( 'SMO', f0, f_ref ) ; return 

fx = f0 ;

HellError = 0.1 ; 0.05; %0.05/sqrt(2) ;
%[scale_var, hl ]= getOptimalScale( f0, HellError ) ;
    [scale_var, hl ]= getOptimalScale2( f0, f_ref, HellError ) ;%f_ref
% HellError = 0.1 ;
% scale_var = getOptimalScale( f0, HellError ) ;
figure(5); title(sprintf('Self-inflation: %f at predefined Hellinger %f',scale_var, HellError)); drawnow ;


% if ( ~isempty(specializedScales) )        
%     fx.covariances = fx.covariances.*specializedScales' ;
% else
    fx.covariances = fx.covariances*scale_var ;
% end

alpha = optimizeWeights( f_ref, fx ) ;  
f0 = pruneMixture( f0, alpha ) ;

% ----------------------------------------------------------------------- %
function f = pruneMixture( f, alpha )

id = find(alpha>0) ;
f.weights = alpha(id)'; 
% f.weights = f.weights(id)/sum(f.weights(id)) ;
f.mu = f.mu(:,id) ;
f.covariances = f.covariances(id,:) ;
