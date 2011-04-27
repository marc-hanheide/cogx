function [mC]=ABSKDEbatch(F,AV)



numC=size(AV,1);
namesAV=1:numC;
[numF,N]=size(F);


[mC,mFS]=KDABNFbatch(F,AV);

ns=10;
Fsd=zeros(numF,ns*numC);
AVsd=zeros(1,ns*numC);

for i=1:numC
   for j=1:numF
      %mFS.AVmeans(i,j)
%      Fsd(j,(i-1)*ns+1:i*ns)=normrnd(mFS.AVmeans(i,j), sqrt(mFS.AVvars(i,j)), 1, ns);
      Fsd(j,(i-1)*ns+1:i*ns) = sampleMixtureOfGaussians( mC(i).mu(j,:), mC(i).weights(j,:), mC(i).covariances(:,j), ns );
      AVsd((i-1)*ns+1:i*ns)=ones(1,ns)*i;
   end
end   
   
Fsd=abs(Fsd);   

weak_learner = tree_node_w(3);
MaxIter=10;
clear mC;

for i=1:numC %for all attribute values
   TrainData=Fsd;
   TrainLabels=[AVsd==i]*2-1;
%   [MLearners MWeights] = ModestAdaBoost(weak_learner, TrainData, TrainLabels, MaxIter);
   [Learners Weights] = RealAdaBoost(weak_learner, TrainData, TrainLabels, MaxIter);

   mC(i).name=namesAV(i);
   mC(i).Learners=Learners;
   mC(i).Weights=Weights;
   mC(i).conf=length(find(AV(i,:)==1));
end;

