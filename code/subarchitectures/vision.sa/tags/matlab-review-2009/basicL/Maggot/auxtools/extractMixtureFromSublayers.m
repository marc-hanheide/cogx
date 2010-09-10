function pdf = extractMixtureFromSublayers( kde )

pdf.Mu = [] ;
pdf.Cov = {} ;
pdf.w = [] ;
num_sublays = length(kde.suffStat.subLayer) ;
for i = 1 : num_sublays
  pdf = mergeDistributions( pdf, kde.suffStat.subLayer(i), [1 kde.w(i)], 0 ) ;      
end