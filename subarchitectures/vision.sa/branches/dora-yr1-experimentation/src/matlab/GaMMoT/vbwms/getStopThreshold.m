function stopThresh = getStopThreshold( covariances, type )
 
allTypes = {'mean', 'min', 'mean_min', 'median', 'median_min'} ;

dim = sqrt(cols(covariances)) ;

% get principal axes of all covariance matrices.
% that is, variances of nonrotated covariance matrices
L = [] ;
mL = [] ;
for i = 1 : rows(covariances)
   C = reshape(covariances(i,:),dim,dim) ; 
   l = svd(C) ;
   L = [L; l] ;
   mL = [mL, min(L)] ;
end

% select minimal observed variance
if ( isequal(type, char(allTypes(1))) )
    minVar = sum(L) ;
elseif ( isequal(type, char(allTypes(2))) )
    minVar = min(L) ;
elseif ( isequal(type, char(allTypes(3))) )
    minVar = mean(mL) ;
elseif ( isequal(type, char(allTypes(4))) )
    minVar = median(L) ;
elseif ( isequal(type, char(allTypes(5))) )
    minVar = median(mL) ;
else
    error('StopThreshold calculation method unknown. Please give a valid method.') ;
end

cutoffThreshold_scale = 1E-2 ;
stopThresh = cutoffThreshold_scale*sqrt(minVar) ;