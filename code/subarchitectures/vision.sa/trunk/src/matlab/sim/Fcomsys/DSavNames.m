function str=DSavNames(avs)

global Coma

numav=length(avs);
str=[];

if numav>0
  if numav==1
     str=cell2mat(Coma.avNames(avs(1)));
  elseif numav==2
    str=[cell2mat(Coma.avNames(avs(1))) ' and ' cell2mat(Coma.avNames(avs(2)))];
  else  
     for i=1:numav-1, str=[str cell2mat(Coma.avNames(avs(i))) ', ']; end;
     str=[str 'and ' cell2mat(Coma.avNames(avs(numav)))];
  end
end  

