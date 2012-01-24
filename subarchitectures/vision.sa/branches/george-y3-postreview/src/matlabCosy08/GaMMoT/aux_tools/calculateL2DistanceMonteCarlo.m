function L2 = calculateL2DistanceMonteCarlo( pdf1, pdf2, N )

% regularize inputs
pdf1 = regularizeform( pdf1 ) ;
pdf2 = regularizeform( pdf2 ) ;
 
% generate a proposal pdf
[X, W] = getProposalPdf( pdf1, pdf2, N ) ;

% evaluate pdf values for pdf1 and pdf2
Y1 = evaluatePdfUniNormMixAt( pdf1, X ) ;
Y2 = evaluatePdfUniNormMixAt( pdf2, X ) ;

L2 = sum(((Y1-Y2).^2).*W)/length(X) ;

% ------------------------------------------------------- %
function [X, W] = getProposalPdf( pdf1, pdf2, N )

pdf.uni.weights = [pdf1.uni.weights, pdf2.uni.weights] ;
pdf.uni.mu = [pdf1.uni.mu, pdf2.uni.mu] ;
pdf.uni.widths = [pdf1.uni.widths; pdf2.uni.widths] ;
pdf.norm.weights = [pdf1.norm.weights, pdf2.norm.weights] ;
pdf.norm.mu = [pdf1.norm.mu, pdf2.norm.mu] ;
pdf.norm.covariances = [pdf1.norm.covariances; pdf2.norm.covariances] ;
w = [pdf.norm.weights, pdf.uni.weights] ;
pdf.norm.weights = pdf.norm.weights / sum(w) ;
pdf.uni.weights = pdf.uni.weights / sum(w) ;

% pdf = pdf1 ;

% sample mixture and evaluate probabilities
X = generateSamplesFromNormUniPdf( pdf, N ) ; 
Y = evaluatePdfUniNormMixAt( pdf, X ) ;
% get sample weights
W = 1./Y ; % W = W / sum(Y) ; %W / sum(W) ;

% m_min = NaN ; m_max = NaN ;
% if ( ~isempty(pdf1.uni.mu) )
%    m_min = min( pdf1.uni.mu - max(pdf1.uni.widths) ) ;
%    m_max = max( pdf1.uni.mu + max(pdf1.uni.widths) ) ;
% else
%    m_min = min( [pdf1.norm.mu - max(sqrt(pdf1.norm.covariances)*3)] ) ; 
%    m_max = max( pdf1.norm.mu + max(sqrt(pdf1.norm.covariances)*3) ) ; 
% end
% 
% if ( ~isempty(pdf1.norm.mu) )
%     m_min = min([m_min, pdf1.norm.mu - max(sqrt(pdf1.norm.covariances)*3)]) ;
%     m_max = max([m_max, pdf1.norm.mu + max(sqrt(pdf1.norm.covariances)*3)]) ;
% end
% 
% X = [m_min:(m_max-m_min)/N:m_max] ; %W = 1/length(X) ;
% W = m_max - m_min ; W = 1/W ;

% ------------------------------------------------------- %
function pdf = regularizeform( pdf )

norm.mu = [] ;  norm.weights = [] ; norm.covariances = [] ;
uni.mu = [] ;  uni.weights = [] ; uni.widths = [] ;
if (~isfield(pdf,'norm') & isfield(pdf,'weights'))
    norm.weights = pdf.weights ;
    norm.covariances = pdf.covariances ;
    norm.mu = pdf.mu ;    
end

if (isfield(pdf,'norm'))
   norm = pdf.norm ; 
end
    
if (~isfield(pdf,'uni'))
    pdf.uni = uni ;
else
    uni = pdf.uni ;
end

if ( size(norm.covariances,3) > 1 )
    norm.covariances = reshape(norm.covariances,length(norm.weights),rows(norm.mu)^2,1) ;
end    
 
pdf = [] ;
pdf.uni = uni ;
pdf.norm = norm ;
