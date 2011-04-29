function pdf = constructKDEfromData( data, varargin )
%
% Matej Kristan (2007)
%
% Constructs a KDE on data and determines the global optimum of the
% resulting distribution.
%
% input: 
%   data ... row of columns. Each coulmn contains one data point.
%   varargin ... additional parameters
%       ('weights', weights) .... weights is row of weights (for each data point)
%       ('reduction', reduction) ... reduction of the kernel bandwidths
%       ('compression', comp_val) ... comp_val = 0,1,2
%                                       0 ... no compression
%                                       1 ... normal compression
%                                       2 ... partitioned compression
%
% out:
%   pdf.mu = means
%   pdf.covariances = covariances
%   pdf.weights = weights
%   pdf.max = maximum of pdf: 
%               pdf.max.pos = location
%               pdf.max.val = value

reduction = 1 ;
compression = 1 ;
weights = 1 ;
reduction = 1 ;
bandwidth_selection = 'silverman' ;
args = varargin;
nargs = length(args);
for i=1:2:nargs
  switch args{i}
      case 'compression', compression = args{i+1} ;
      case 'weights', weights = args{i+1} ;
      case 'reduction', reduction = args{i+1} ;
      case 'type', bandwidth_selection = args{i+1} ;
      case 'reduction', reduction = args{i+1} ;
      otherwise, error(['unrecognized argument ' args{i}])
  end
end

% estimate KDE from data
pdf = estimateKDEfromData( data, 'weights', weights, 'reduction', reduction ) ;

pdf.covariances = pdf.covariances*reduction ;

% if requested, compress the distribution
pdf = compressMe( pdf, compression ) ;

% find the maximum-probability point on the pdf
[max_pos, max_val] = findGlobalMaximum( pdf ) ;
pdf.max.pos = max_pos ;
pdf.max.val = max_val ;

% ------------------------------------------------------------ %
function pdf = compressMe(pdf, compression )

if compression == 0 % no compression
    return ;
elseif compression == 1 % normal compression 
    pdf = compressDistribution( pdf, 'showIntermediate', 0  ) ;
elseif compression == 2 % incremental-normal compression
    pdf = incrementalCompression( pdf, 'showIntermediate', 0 ) ;
elseif compression == 3
    pdf = incrementalSubOptCompression( pdf, 'showIntermediate', 0  ) ;
end
    
 