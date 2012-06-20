function mC1=KDBFcompress(mC)

nsbf=1000; %parameter for belFunction
numC=length(mC);

mC1=mC;
for i=1:numC
   pdf1=mC(i);
   components = pdf1.components ;
   pdf1 = compressDistribution( pdf1, 'showIntermediate', 0) ;
   pdf1.components = components;
   pdf1.belFun=calcBelFun(pdf1,nsbf);

   mC1(i).mu=pdf1.mu;
   mC1(i).covariances=pdf1.covariances;
   mC1(i).weights=pdf1.weights;
   mC1(i).belFun=pdf1.belFun;
   mC1(i).conf=mC(i).conf;
   mC1(i).components=pdf1.components;


end
