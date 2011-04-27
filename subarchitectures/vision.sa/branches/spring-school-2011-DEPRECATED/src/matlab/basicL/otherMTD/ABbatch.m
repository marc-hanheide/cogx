function [mC]=ABbatch(F,AV)
%[mC,mDA,mFS]=MVBFbatch(X,AV)
%MVBF batch learning.
%X: input training images
%AV: given AVs
%mC: model of AVs
%mDA: model of detected attributes
%mFS: feature statistics



numC=size(AV,1);
namesAV=1:numC;
%N=size(F,2);


weak_learner = tree_node_w(3);
MaxIter=10;


for i=1:numC %for all attribute values
   TrainData=F;
   TrainLabels=[AV(i,:)==1]*2-1;
%   [MLearners MWeights] = ModestAdaBoost(weak_learner, TrainData, TrainLabels, MaxIter);
   [Learners Weights] = RealAdaBoost(weak_learner, TrainData, TrainLabels, MaxIter);

   mC(i).name=namesAV(i);
   mC(i).Learners=Learners;
   mC(i).Weights=Weights;
   mC(i).conf=length(find(AV(i,:)==1));
end;

