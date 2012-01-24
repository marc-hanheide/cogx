function AVs=lf2sf(AVl)
%AVs=lf2sf(AVl)
%Long ordered format to short format of AVs.
%AVl: AVs in long ordered format.
%AVs: AVs in short format.


N=size(AVl,2);

maxAV=max(sum(AVl>0));
AVs=zeros(maxAV,N);

for i=1:N
  av=find(AVl(:,i));
  AVs(1:length(av),i)=av;
end;

  