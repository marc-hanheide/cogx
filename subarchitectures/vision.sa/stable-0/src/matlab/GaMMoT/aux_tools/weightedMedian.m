function m = weightedMedian( data, weights )
% calculates median of data. If weigts are given, then weighted median is
% calculated instead.

if length(weights) < length(data)
   m = median(data) ;
else
  [data, id] = sort(data, 'ascend') ;
  weights = weights(id) ;
  cum = cumsum(weights) ;
  cum = cum / max(cum) ;
  [value, id] = min(abs(cum-0.5)) ;
  m = data(id) ;
end