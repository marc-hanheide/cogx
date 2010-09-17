function pdf_res = unlearnAndCompress( pdf_pos, pdf_neg, varargin )
%
% Matej Kristan (2007)
%
% Unlearns distribution pdf_pos using pdf_neg and compresses the result.
%
 
reportProgress = 0 ;
hellErrorGlobal = 0.1 ;
scaleErrorThreshold = 1 / 0.7 ;
args = varargin;
nargs = length(args);
for i=1:2:nargs
    switch args{i}
        case 'hellErrorGlobal', hellErrorGlobal = args{i+1} ;
        case 'scaleErrorThreshold', scaleErrorThreshold = args{i+1} ;
        case 'reportProgress', reportProgress = args{i+1} ;
    end
end

if isfield( pdf_pos, 'pars2' )
    hellErrorGlobal = pdf_pos.pars2.hellErrorGlobal ;
    scaleErrorThreshold = pdf_pos.pars2.scaleErrorThreshold ;
end

pdf_pos_components = pdf_pos.components ;
pdf_neg_components = pdf_neg.components ;

e = gaussUnlearn( pdf_pos, pdf_neg ) ;
e = pruneDistribution( e ) ;
% c = fragmentNegativeComponents( e ) ;
 
pdf_res = updateIKDE( e,[], 'reportProgress', reportProgress, 'modify', 1,'showIntermediate', reportProgress ) ;
pdf_res = updateIKDE( pdf_res,[], 'reportProgress', reportProgress ) ; 

% pdf_res = compressDistribution( e, 'showIntermediate', 1, 'modify', 1,...
%                                'hellErrorGlobal', hellErrorGlobal,...
%                                'scaleErrorThreshold', scaleErrorThreshold ) ;
 
% find the maximum-probability point on the pdf
[max_pos, max_val] = findGlobalMaximum( pdf_res ) ;
pdf_res.max.pos = max_pos ;
pdf_res.max.val = max_val ;
pdf_res.components = pdf_pos_components + pdf_neg_components ;

% ---------------------------------------------------- %
function pdf1 = fragmentNegativeComponents( pdf )

select = find( pdf.weights<0) ;
pdf1 = pdf ;

mu = pdf.mu(select) ;
C = pdf.covariances(select) ;
w = pdf.weights(select) ;

d = sqrt( C )*0.5 ;
mu2 = [mu - d',mu + d' ];
C2 = [C;C]*sqrt(0.5) ;
w2 = [w, w]*0.5 ;

select1 = find( pdf.weights>=0 ) ;
pdf1.mu = [mu2, pdf.mu(select1) ] ;
pdf1.covariances = [C2; pdf.covariances(select1)];
pdf1.weights = [w2, pdf.weights(select1) ];

% ----------------------------------------------------- %
function e = pruneDistribution( e ) 

select = find( abs(e.weights)>1e-12 ) ;
e.mu = e.mu(:,select) ;
e.covariances = e.covariances(select,:) ;
e.weights = e.weights(select) ;



