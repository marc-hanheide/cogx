function belFun=calcBelFun(pdf,ns)

%X = sampleMixtureOfGaussians( pdf.mu, pdf.weights, pdf.covariances, ns ) ;
X = sampleUniformMixtureOfGaussians( pdf.mu, pdf.covariances, ns ) ;
vals = evaluateDistributionAt( pdf.mu, pdf.weights, pdf.covariances, X ) ;
%nvals=vals/pdf.max.val;
nvals=vals;
% idxs=find(nvals>1);
% nvals(idxs)=1-rand(1,length(idxs))*1e-3;

snvals = sort(nvals) ;

csnvals=zeros(size(snvals));
for i=2:length(snvals)
   csnvals(i)=csnvals(i-1)+snvals(i);
end;   
ncsnvals=csnvals/csnvals(end);

belFun=[snvals;ncsnvals];

