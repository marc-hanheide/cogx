function CC=c2cc(C,CM)
%function CC=c2cc(C,CM)
%From old ordered long format C to new CC

numSC=max(CM(:,2));
%N=size(C,2);

for i=1:numSC
   ci=find(CM(:,2)==i);
   CC{i}=C(CM(ci,1),:);
end
