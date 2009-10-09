function pdf = mixUniNormMixtures( pdf1, pdf2 )

pdf1 = fixPdf( pdf1 ) ;
pdf2 = fixPdf( pdf2 ) ;

pdf.uni.weights = [pdf1.uni.weights, pdf2.uni.weights] ;
pdf.uni.mu = [pdf1.uni.mu, pdf2.uni.mu] ;
pdf.uni.widths = [pdf1.uni.widths; pdf2.uni.widths] ;

pdf.norm.weights = [pdf1.norm.weights, pdf2.norm.weights] ;
pdf.norm.mu = [pdf1.norm.mu, pdf2.norm.mu] ;
pdf.norm.covariances = [pdf1.norm.covariances; pdf2.norm.covariances] ;

% normalize mixture
w1 = 0 ;
if( ~isempty(pdf.uni.weights) )
    w1 = sum(pdf.uni.weights) ;
end

w2 = 0 ;
if( ~isempty(pdf.norm.weights) )
    w2 = sum(pdf.norm.weights) ;
end

w = w1 + w2 ;
if( ~isempty(pdf.uni.weights) )
    pdf.uni.weights = pdf.uni.weights / w ;
end
if( ~isempty(pdf.norm.weights) )
    pdf.norm.weights = pdf.norm.weights / w ;
end


    

% --------------------------------------------------- %
function pdf = fixPdf( pdf0 )

if (~isfield(pdf0,'norm') && ~isfield(pdf0,'uni'))
    pdf.norm = pdf0 ; 
    pdf.uni.mu = [] ; pdf.uni.weights = [] ; pdf.uni.widths = [] ;
elseif (~isfield(pdf0,'uni') & isfield(pdf0,'norm'))
    pdf.norm = pdf0.norm ;     
    pdf.uni.mu = [] ; pdf.uni.weights = [] ; pdf.uni.widths = [] ;    
elseif(~isfield(pdf0,'norm') & isfield(pdf0,'uni'))    
    pdf.uni = pdf0.uni ;
    pdf.norm.mu = [] ; pdf.norm.weights = [] ; pdf.norm.covariances = [] ;
else    
    pdf = pdf0 ;
end