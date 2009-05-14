function [mean_spread, median_spread] = getSpreadOfpdfs( pdf_array, varargin ) 
% Matej Kristan (2007)
%
% Evaluates measure of spread between the distributions in pdf_array.
% input:
%   pdf_array ... array of pdfs, e.g. {pdf1, pdf2, pdf3,...}
%   varargin  ... optional additional parameters
%                   ('weightsPdf',weightsPdf) ... weight of each
%                   distribution
%

weightsPdf = 1 ;
args = varargin ;
nargs = length(args) ;
for i=1:2:nargs
  switch args{i}
   case 'weightsPdf', weightsPdf = args{i+1} ; 
   otherwise, error(['unrecognized argument ' args{i}]) ; 
  end
end

num_pdfs = length(pdf_array) ;
if ( length(weightsPdf) == 1 ) weightsPdf = ones(1,num_pdfs) ; end
weightsPdf = weightsPdf/sum(weightsPdf) ;

% get mean pdf
pdf_mean.mu = [] ;
pdf_mean.covariances = [] ;
pdf_mean.weights = [] ;
for i = 1 : num_pdfs
    pdf_mean.mu = [pdf_mean.mu, pdf_array{i}.mu] ;
    pdf_mean.covariances = [pdf_mean.covariances; pdf_array{i}.covariances] ;
    pdf_mean.weights = [pdf_mean.weights, (pdf_array{i}.weights)*weightsPdf(i)] ;
end

% get distances from the mean
d = zeros(1,num_pdfs) ;
for i = 1 : num_pdfs
    d(i) = suHellinger( pdf_mean, pdf_array{i} ) ;
end

% get mean spread
mean_spread = sqrt(sum((d.^2).*weightsPdf)) ;

% get median spread
median_spread = weightedMedian( d, weightsPdf ) ;
