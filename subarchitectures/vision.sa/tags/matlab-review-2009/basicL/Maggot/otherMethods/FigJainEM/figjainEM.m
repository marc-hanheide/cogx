function pdf = figjainEM( data , mink, maxk, regularize, minth, convoption )
% wrapper for FigJain batch EM

convoption = 0 ;
minth = 1e-4 ;
regularize = 1e-5 ;
[bestk,bestpp,bestmu,bestcov,dl,countf] = mixtures4(data,mink,maxk,regularize,minth,convoption) ;

pdf.w = bestpp ;
pdf.Mu = bestmu ;
pdf.Cov = {} ;
for i = 1 : length(pdf.w)
    pdf.Cov = horzcat(pdf.Cov, {bestcov(:,:,i)}) ;
end