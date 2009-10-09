function bel=getBel(pdf,data)

p = evaluateDistributionAt( pdf.mu, pdf.weights, pdf.covariances, data ) ;
bel=p;
%  bel = p/pdf.max.val ;
%  bel = (bel<1).*bel + (bel>=1);

lenBf=size(pdf.belFun,2);
i=1;
while i<lenBf && bel>pdf.belFun(1,i)
      i=i+1;
end
bel=pdf.belFun(2,i);