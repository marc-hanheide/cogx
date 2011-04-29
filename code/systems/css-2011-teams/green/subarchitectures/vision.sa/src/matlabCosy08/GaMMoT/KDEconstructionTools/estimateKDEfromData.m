function [pdf, C_curr]= estimateKDEfromData( data, varargin )
% data ... a row of column vectors of data features (one sample per column)
% pdf  ... output distribution
%           .mu = mean vectors (data)
%           .covariances = covariances reshaped to rows
%           .weights = weights of components (one per column)

C_curr = [] ;
bwmethod = 'Silverman' ; % 'Plugin'
weights = 1 ;
reduction = 1.0 ;  
args = varargin;
nargs = length(args);
for i=1:2:nargs
  switch args{i}
   case 'weights', weights = args{i+1} ;
   case 'reduction', reduction = args{i+1} ;
   case 'bwmethod', bwmethod = args{i+1} ;
   otherwise, error(['unrecognized argument ' args{i}]) ;
  end
end

N = cols(data) ;

if isequal(bwmethod,'Silverman')
    Covariance = getSilvermanCovarianceFromData( data, varargin{:} ) ;
elseif isequal(bwmethod,'Plugin')
    pdf = get_1d_OptimalKDE( data ) ;
    return ;
end


if ( length(weights) == 1 ) weights = ones(1,N)/N ; end   

d = rows(data) ;
pdf.mu = data ;
pdf.covariances = repmat( reshape(Covariance,1,d*d),N,1) ;
pdf.weights = weights ;
