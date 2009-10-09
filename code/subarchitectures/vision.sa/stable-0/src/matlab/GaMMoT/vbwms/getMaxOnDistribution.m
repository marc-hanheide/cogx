function pdf = getMaxOnDistribution( pdf )

% find the maximum-probability point on the pdf
[max_pos, max_val] = findGlobalMaximum( pdf ) ;
pdf.max.pos = max_pos ;
pdf.max.val = max_val ;
