function [mAV]=ABbatch(F,AV)
%[mAV,mDA,mFS]=MVBFbatch(X,AV)
%MVBF batch learning.
%X: input training images
%AV: given AVs
%mAV: model of AVs
%mDA: model of detected attributes
%mFS: feature statistics



numAV=size(AV,1);
namesAV=1:numAV;
%N=size(F,2);


weak_learner = tree_node_w(3);
MaxIter=10;


for i=1:numAV %for all attribute values
   TrainData=F;
   TrainLabels=[AV(i,:)==1]*2-1;
%   [MLearners MWeights] = ModestAdaBoost(weak_learner, TrainData, TrainLabels, MaxIter);
   [Learners Weights] = RealAdaBoost(weak_learner, TrainData, TrainLabels, MaxIter);

   mAV(i).name=namesAV(i);
   mAV(i).Learners=Learners;
   mAV(i).Weights=Weights;
   mAV(i).conf=length(find(AV(i,:)==1));
end;

