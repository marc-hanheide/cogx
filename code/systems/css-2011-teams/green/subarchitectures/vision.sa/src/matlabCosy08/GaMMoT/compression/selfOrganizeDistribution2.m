function f_dest = selfOrganizeDistribution2( f_ref, f_dest )

maxIterations = 10 ;

% figure(1); clf; debugShowX( f_ref, f_ref ) ;drawnow ;
tic
% remove units
if ( isempty(f_dest) ) f_dest = f_ref ; end 

numComps = length(f_ref.weights) ;
for i_org = 1 : maxIterations
    
    disp('removing units')
    % remove units
    f_dest = removeUnits( f_dest, f_ref, 2 ) ;

    disp('calculating ownership')
    % find ownerships
    ownerships = getOwnerships2( f_ref, f_dest ) ;

    % what would happen if we recalculated the new components via
    % weighted ownerships?
    
    disp('reestimating parameters')
    % reestimate parameters
    f_dest = reEstimateParameters( f_ref, f_dest, ownerships ) ;
 
%       debugShowX( f_ref, f_dest ) ; drawnow ;
    
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

w = sum(ownerships,1) ;
ownerships = ownerships./repmat(w,len_dest,1) ;

fr_0 = f_ref ;
valid = ones(1,len_dest) ;
len_dest = length(f_dest.weights) ;
    for j = 1 : len_dest
        %f0.weights = f_dest.weights ;
                
%         id = find(ownerships==j) ;
%         if isempty(id)
%             f_dest.weight(j) = 0 ;
%             continue ;
%         end
%         
%         fr_0.mu = f_ref.mu(:,id) ;
%         fr_0.weights = f_ref.weights(id) ;
%         fr_0.covariances = f_ref.covariances(:,:,id) ;
        
        own = ownerships(j,:) ; own = own / sum(own) ;  
        fr_0.weights = f_ref.weights.*own ; ww = sum(fr_0.weights) ;
        fr_0.weights = fr_0.weights / sum(fr_0.weights) ;
           
        fd_0.weights = sum(fr_0.weights) ;
        fd_0.mu = sum(fr_0.mu .* fr_0.weights)/sum(fr_0.weights) ;
        cc = reshape(fr_0.covariances,1,length(fr_0.weights)) ;
        fd_0.covariances = sum(fr_0.weights.*(cc + (fd_0.mu-fr_0.mu).^2))/sum(fr_0.weights) ;
        
     %   fd_0 = refitGaussianMixtureFunApproxSingle( fd_0, fr_0 ) ;
     fd_0.weights = ww ;
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
