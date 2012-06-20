function rAVc=lf2lof(rAV,numAVgt)
%rAVc=luof2lf(rAV,mAV,numAVgt)
%Long unordered format to long ordered format
%rAV: recognized AVs in long unordered format (output of MVBFrecAV)
%mAV: models of AV
%numAVgt: number of all AV
%rAVc: all AVs in a long ordered format
%Extends the rAV matrix with additional rows containing -1 (i.e., ANSdk) for AV that have not
%been learned yet.

readConstants;

names=rAV(:,1);
N=size(rAV,2)-1;

rAVc=ones(numAVgt,1+N)*ANSdk;
rAVc(:,1)=1:numAVgt;
for i=1:length(names)
  rAVc(names(i),:)=rAV(i,:);
end;


