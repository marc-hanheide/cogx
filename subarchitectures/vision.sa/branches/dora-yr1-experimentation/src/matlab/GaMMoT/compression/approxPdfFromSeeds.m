function pdf_t = approxPdfFromSeeds( pdf_ref, seeds, MaxIterations, weights )
%
% Approximate the pdf_ref GMM using seeds
%
% Author: Matej Kristan 2008
%

debugShow = 0 ;
% MaxIterations = 1 ;

% get dimensions
lenSeeds = cols(seeds) ;
lenRefs = length(pdf_ref.weights) ;
d = rows(pdf_ref.mu) ;

% initialize target pdf
pdf_t.weights = zeros(1, lenSeeds) ;
pdf_t.mu = zeros(d, lenSeeds) ;
pdf_t.covariances = zeros(lenSeeds, d*d) ;

% initialize reference pdfs
f_ref_i = pdf_ref ;
ref_weights = pdf_ref.weights ;
% reshape covariances into blocks
f_ref_i.covariances = reshape(f_ref_i.covariances',d,d,lenRefs) ;

for iteration = 1 : MaxIterations

    % get ownerships from weights
    [ownerships, norms] = getWeightedOwnerships( pdf_ref, seeds, weights  ) ;

    % Start minimization of errors by subparts w.r.t. seeds
    for i = 1 : lenSeeds
        % construct i-th reference sub distribution
        f_ref_i.weights = ref_weights.*(ownerships(:,i)') ;

        % approximate the i-th modified reference mixture model
        f_i = refitSingleGaussianByFunApprox( f_ref_i ) ;

        % show for debugging
        dShow(f_ref_i, f_i, pdf_t, 0, debugShow ) ; 

        % update destination mixture
        pdf_t.mu(:,i) = f_i.mu ;
        pdf_t.covariances(i,:) = f_i.covariances ;
        pdf_t.weights(i) = f_i.weights ;
    end

    pdf_t.weights = pdf_t.weights / sum(pdf_t.weights) ;
    % reshape covariances back to lines
    pdf_t.covariances = reshape(pdf_t.covariances,lenSeeds,d^2,1) ;

    % show for debugging
    dShow(pdf_ref, [], pdf_t, 1, debugShow ) ; 
    seeds = pdf_t.mu ;

end




function dShow(f_ref_i, f_i, pdf_t, compTo, debugShow )

if ( debugShow == 0 ) return ; end
clf ;

d = rows(f_ref_i.mu) ;
lenSeeds = cols(f_ref_i.weights) ;
f_ref_i.covariances = reshape( f_ref_i.covariances, lenSeeds, d^2, 1 ) ;
bounds = showDecomposedPdf( f_ref_i, 'linTypeSum', 'r-', 'returnBounds',1 ) ;

if (compTo == 0)
    showDecomposedPdf( f_i, 'bounds', bounds, 'linTypeSum', 'c:' ) ;
else
    lenSeeds = cols(pdf_t.weights) ;
    pdf_t.covariances = reshape( pdf_t.covariances, lenSeeds, d^2, 1 ) ; 
    showDecomposedPdf( pdf_t, 'bounds', bounds, 'linTypeSum', 'g-', 'linTypeSub', 'c:' ) ;
end
drawnow ;
    
