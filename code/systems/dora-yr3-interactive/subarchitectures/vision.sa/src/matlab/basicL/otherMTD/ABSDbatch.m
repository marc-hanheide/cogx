function [mC]=ABSDbatch(F,AV)
%[mC,mDA,mFS]=MVBFbatch(X,AV)
%MVBF batch learning.
%X: input training images
%AV: given AVs
%mC: model of AVs
%mDA: model of detected attributes
%mFS: feature statistics



numC=size(AV,1);
namesAV=1:numC;
[numF,N]=size(F);


[mC,mDA,mFS]=MVBFbatch(F,AV);

ns=1000;
Fsd=zeros(numF,ns*numC);
AVsd=zeros(1,ns*numC);

for i=1:numC
   for j=1:numF
      %mFS.AVmeans(i,j)
      Fsd(j,(i-1)*ns+1:i*ns)=normrnd(mFS.AVmeans(i,j), sqrt(mFS.AVvars(i,j)), 1, ns);
      AVsd((i-1)*ns+1:i*ns)=ones(1,ns)*i;
   end
end   
   
Fsd=abs(Fsd);   

weak_learner = tree_node_w(3);
MaxIter=10;


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

