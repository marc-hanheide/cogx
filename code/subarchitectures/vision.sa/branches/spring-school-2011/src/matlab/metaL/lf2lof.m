function rAVc=lf2lof(rAV,numCgt)
%rAVc=luof2lf(rAV,mC,numCgt)
%Long unordered format to long ordered format
%rAV: recognized AVs in long unordered format (output of MVBFrecAV)
%mC: models of AV
%numCgt: number of all AV
%rAVc: all AVs in a long ordered format
%Extends the rAV matrix with additional rows containing -1 (i.e., ANSdk) for AV that have not
%been learned yet.

readConstants;

names=rAV(:,1);
N=size(rAV,2)-1;

rAVc=ones(numCgt,1+N)*ANSdk;
rAVc(:,1)=1:numCgt;
for i=1:length(names)
  rAVc(names(i),:)=rAV(i,:);
end;


