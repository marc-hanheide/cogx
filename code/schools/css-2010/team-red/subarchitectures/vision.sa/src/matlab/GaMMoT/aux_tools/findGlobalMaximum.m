function [y_max, a] = findGlobalMaximum(r)
% Function searches for a global maximum in the gaussian mixture
% Mean Shift is used to locate optima and the uptimum with
% highest probability is chosen as the result.

% get local maximums
stopThresh = getStopThreshold( r.covariances, 'median_min' ) ;
[precisions, determinants] = getPrecisionsAndDets( r.covariances ) ;
[ centers, id_converged ] = ...
            vbwmsModeCandidates( r.mu, r.weights, r.covariances,...
                                 precisions, determinants, stopThresh, [] ) ;
% find global maximum
y_evals = evaluateDistributionAt( r.mu, r.weights, r.covariances, centers ) ;
[a,i_pos] = max(y_evals) ;
y_max = centers(:,i_pos) ;

if nargout == 1
    a = [] ;
end