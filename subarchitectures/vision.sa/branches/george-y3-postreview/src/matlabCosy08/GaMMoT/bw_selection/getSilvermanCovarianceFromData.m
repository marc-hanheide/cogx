function Covariance = getSilvermanCovarianceFromData( data, varargin )
% Matej Kristan (2007) 
%
% Creates multi dimensional kernel bandwidths.
% data is column-wise vector

weights = 1 ;
reduction = 1.0 ;  
args = varargin;
nargs = length(args);
for i=1:2:nargs
  switch args{i}
   case 'weights', weights = args{i+1} ;
   case 'reduction', reduction = args{i+1} ;
   case 'bwmethod',  ;    
   otherwise, error(['unrecognized argument ' args{i}]) ;
  end
end

bandwidth = [] ;
for i = 1 : size(data,1)
    bandwidth(i) = bandwidthEstimate1D( data(i,:), weights ) ;
end

bandwidth = bandwidth*reduction ; 
Covariance = diag(bandwidth) ;
% [bandwidth, validity ] = regularizeBandwidth( bandwidth ) ;
% if nargout == 2 
%     valid = validity ;
% end

%--------------------------------------------------------------------------
function [bandwidth, valid ]= regularizeBandwidth( bandwidth, default_min_bwidth )
% check whether dimensions are greater than zero and set those that are
% zero to the minimal bandwidth

valid = 1 ; 
if isempty(find(bandwidth > default_min_bwidth)) 
    valid = 0 ;
end

idx = find(bandwidth > default_min_bwidth) ;
if ( isempty(idx) )
    min_band = default_min_bwidth ;
else
    min_band = min(bandwidth(idx)) ;
end

idx = find(bandwidth <=default_min_bwidth) ;
if ( ~isempty(idx) )
    bandwidth(idx) = min_band ;
end

% --------------------------------------------------------------- %
function bandwidth = bandwidthEstimate1D( data, weights )
% Calculates a robust estimate of the 1D kernel bandwidth.
% If weights are present, then the medians are calculated via weighted
% median.
   
% m = weightedMedian(data, weights) ;    
% S = (weightedMedian(abs(data-m),weights)/0.6745)^2 ;

S = (var(data)); 
N = size(data,2) ;
bandwidth = getSilvermanBWfromGaussian( S, N ) ;
 
% --------------------------------------------------------------- %


