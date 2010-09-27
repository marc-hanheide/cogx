function str=idx2name(idx,names)

%idx
%names
numI=length(idx);
str=[];
for i=1:numI
    try 
        str=[str ' ' names(idx(i),:)];
    catch exceprtion
        str=[str ' ???' idx(i)];
    end
end    
    
