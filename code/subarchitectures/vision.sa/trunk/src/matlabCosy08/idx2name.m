function str=idx2name(idx,names)

numI=length(idx);
str=[];
for i=1:numI
    str=[str ' ' names(idx(i),:)];
end    
    