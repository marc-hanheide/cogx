function AVl=sf2lf(AVs,numAV)
%AVl=sf2lf(AVs,numAV)
%Short format to long format of AVs.
%AVs: AVs in short format
%numAV: number of all attribute values
%AVl: AVs in long format

if nargin<2 numAV=max(AVs(:)); end;

N=size(AVs,2);

AVl=zeros(numAV,N);

for i=1:N
  AVl(AVs(:,i),i)=1;
end;  
  