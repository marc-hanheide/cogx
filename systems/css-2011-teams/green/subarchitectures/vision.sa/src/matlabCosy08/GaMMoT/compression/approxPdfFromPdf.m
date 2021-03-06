function pdf_t = approxPdfFromPdf( pdf_ref, pdf_dest, MaxIterations )
%
% Approximate the pdf_ref GMM using seeds
%
% Author: Matej Kristan 2008
%

debugShow = 0 ;
% MaxIterations = 1 ;

% get dimensions
lenSeeds = cols(pdf_dest.weights) ;
lenRefs = length(pdf_ref.weights) ;
d = rows(pdf_ref.mu) ;

% initialize target pdf
pdf_t = pdf_dest ;

% initialize reference pdfs
f_ref_i = pdf_ref ;
ref_weights = pdf_ref.weights ;
% reshape covariances into blocks
f_ref_i.covariances = reshape(f_ref_i.covariances',d,d,lenRefs) ;

for iteration = 1 : MaxIterations

    % get ownerships from weights
    [ownerships, normsXXX] = getWeightedOwnerships( pdf_ref, [], pdf_t ) ;

    % remove components with zero weights
    [ownerships, lenSeeds, pdf_t] = pruneDestination( pdf_t, ownerships) ;
    
    % Start minimization of errors by subparts w.r.t. seeds
    for i = 1 : lenSeeds

%         id = find([1:length(pdf_dest.weights)] ~= i) ;
%         f_j.weights = [pdf_dest.weights, -pdf_t.weights(id) ] ;
%         f_j.mu = [pdf_dest.mu, pdf_t.mu(:,id) ] ;
%         f_j.covariances = reshape([pdf_dest.covariances; pdf_t.covariances(id,:)], 1,1, length(f_j.weights) );
%         
%         
%         % approximate the i-th modified reference mixture model
%         f_i = refitSingleGaussianByFunApprox( f_j ) ;
        

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


function [ownerships, lenSeeds, pdf_dest] = pruneDestination( pdf_dest, ownerships) 

O = sum(ownerships,1) ;
I = find(O > 0) ;
pdf_dest.weights = pdf_dest.weights(I) ;
pdf_dest.mu = pdf_dest.mu(:,I) ;
pdf_dest.covariances = pdf_dest.covariances(I,:) ;

ownerships = ownerships(:,I) ;
lenSeeds = length(I) ;



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
    
