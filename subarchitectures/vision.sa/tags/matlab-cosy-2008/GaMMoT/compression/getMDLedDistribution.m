function f_dest = getMDLedDistribution( f_ref, f_dest )

f_ref.covariances = reshape(f_ref.covariances,1,1,length(f_ref.weights)) ;
f_dest.covariances = reshape(f_dest.covariances,1,1,length(f_dest.weights))   ;


% [f_dest, oneToOneCoherence] = combineComponents( f_ref ) ;

[f_dest, oneToOneCoherence] = combineComponentsFullMDL( f_ref, f_ref ) ; 

f_ref.covariances = reshape(f_ref.covariances,length(f_ref.weights),1) ;
f_dest.covariances = reshape(f_dest.covariances,length(f_dest.weights),1) ;

ownerships = getOwnerships( f_ref, f_dest ) ;

    % what would happen if we recalculated the new components via
    % weighted ownerships?
    
    disp('reestimating parameters')
    % reestimate parameters
    f_dest = reEstimateParameters( f_ref, f_dest, ownerships ) ;


    
    
return