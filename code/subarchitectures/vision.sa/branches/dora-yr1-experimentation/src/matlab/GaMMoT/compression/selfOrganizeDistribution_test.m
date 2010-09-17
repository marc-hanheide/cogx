function f_dest = selfOrganizeDistribution_test( f_ref, seeds )

maxIterations = 10 ;


bounds = [-2 1 -1 1] ;

% seeds = [] ;
% lenSeed = 3; ceil(length(f_ref.weights)/3) ;
% figure(2); clf ;
% showPdfEstimateResults( [], [], f_ref.mu, f_ref.weights, f_ref.covariances, bounds, [], [] ) ;
% for i = 1 : lenSeed
%    X = ginput(1) ;
%    plot(X(1),0,'*r') ;    
%    seeds = [seeds, X(1)] ;
% end


f_dest = f_ref ;
lenComps0 = length(f_dest.weights)+1 ;
while lenComps0 ~= length(f_dest.weights) ; 
    
    lenComps0 = length(f_dest.weights) ;
    f_dest = removeUnits( f_dest, f_ref, 2 ) ;
    
   
%     seeds = f_dest.mu ;
%     weights = f_dest.weights ;
%     f_dest = approxPdfFromSeeds( f_ref, seeds, 1, weights ) ;
     
    f_dest.weights =  abs(f_dest.weights) ; f_dest.weights = f_dest.weights / sum(f_dest.weights);
    f_dest = approxPdfFromPdf( f_ref, f_dest, 1 ) ;
   
%     figure(4); clf;
%     showPdfEstimateResults( [], [], f_ref.mu, f_ref.weights, f_ref.covariances, bounds, [], [] ) ;
%     showPdfEstimateResults( [], [], f_dest.mu, f_dest.weights, f_dest.covariances, bounds, [], [] ) ;
%     pause
end

% f_dest = approxPdfFromSeeds( f_ref, seeds, 5 ) ;
%     figure(5); clf;
%     showPdfEstimateResults( [], [], f_ref.mu, f_ref.weights, f_ref.covariances, bounds, [], [] ) ;
%     showPdfEstimateResults( [], [], f_dest.mu, f_dest.weights, f_dest.covariances, bounds, [], [] ) ;
%     pause

return ;

[ownerships, norms] = getWeightedOwnerships( f_ref, seed ) ;
% These are weights to reweight the target distribution for reweighting
% the target distribution and individualizing it to a specific seed

figure(3); clf ; 
subplot(lenSeed+1,1,1) ; showPdfEstimateResults( [], [], f_ref.mu, f_ref.weights, f_ref.covariances, bounds, [], [] ) ;
f_combined.mu = [] ; f_combined.covariances = [] ; f_combined.weights = [] ;
for i = 1 : lenSeed
    f_ref_i = reweightMixture( f_ref, ownerships(:,i)' ) ;
    subplot(lenSeed+1,1,i+1) ;
    showPdfEstimateResults( [], [], f_ref_i.mu, f_ref_i.weights, f_ref_i.covariances, bounds, [], [] ) ;
    hold on ; plot(seed(i),0,'*r') ;
    f_combined.mu = [f_combined.mu, f_ref_i.mu] ;
    f_combined.covariances = [f_combined.covariances; f_ref_i.covariances] ;
    f_combined.weights = [f_combined.weights, f_ref_i.weights] ;
end
f_combined.weights = f_combined.weights / sum(f_combined.weights) ;

figure(4); clf; 
showPdfEstimateResults( [], [], f_ref.mu, f_ref.weights, f_ref.covariances, bounds, [], [] ) ;
showPdfEstimateResults( [], [], f_combined.mu, f_combined.weights, f_combined.covariances, bounds, [], [] ) ;




return


function f_ref = reweightMixture( f_ref, w_mod )
f_ref.weights = f_ref.weights.*w_mod ;
%f_ref.weights = f_ref.weights / sum(f_ref.weights) ;




return




% what would happen if we recalculated the new components via
% weighted ownerships?

disp('reestimating parameters')
% reestimate parameters
f_dest = reEstimateParameters( f_ref, f_dest, ownerships ) ;


numComps = length(f_ref.weights) ;
for i_org = 1 : maxIterations
    
    disp('removing units')
    % remove units
    f_dest = removeUnits( f_dest, f_ref, 2 ) ;
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

valid = ones(1,len_dest) ;
len_dest = length(f_dest.weights) ;
    for j = 1 : len_dest
        f0.weights = f_dest.weights ;
        
        
        id = find(ownerships==j) ;
        if isempty(id)
            f_dest.weight(j) = 0 ;
            continue ;
        end
        
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

% ----------------------------------------------------------------------- %
function f0 = reoptimizeWeights( f0, f_ref)
alpha = optimizeWeights( f_ref, f0 ) ;
f0.weights = alpha' ;

% ----------------------------------------------------------------------- %
function f0 = removeUnits( f0, f_ref, scale_var )
%scale_var = 2 ; 4 ; % 1.5^2 ;

HellError = 0.2 ;
    scale_var = getOptimalScale( f0, HellError ) ;
    figure(5); title(sprintf('Self-inflation: %f at predefined Hellinger %f',scale_var, HellError)); drawnow ;
 
 
fx = f0 ;
fx.covariances = fx.covariances*scale_var ;
alpha = optimizeWeights( f_ref, fx ) ;
f0 = pruneMixture( f0, alpha ) ;

% ----------------------------------------------------------------------- %
function f = pruneMixture( f, alpha )

id = find(alpha>0) ;
f.weights = alpha(id)'; 
% f.weights = f.weights(id)/sum(f.weights(id)) ;
f.mu = f.mu(:,id) ;
f.covariances = f.covariances(id,:) ;
