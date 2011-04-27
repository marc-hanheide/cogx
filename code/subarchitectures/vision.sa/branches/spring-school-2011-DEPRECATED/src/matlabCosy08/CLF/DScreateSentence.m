function sentence=DScreateSentence(data)

global senMap

if isempty(data)
   sentence=[];
else
   beginSen=find(data<0);
   if length(beginSen)<2 %one sentence

      req=-data(1);
      if length(data)>1
         avs=data(2:end);
      else
         avs=[];
      end;

      sentence=senMap{req,1};
      if ~isempty(senMap{req,2})
         sentence=[sentence DSavNames(avs) senMap{req,2}];
      end

   else %multiple sentences
      sentence=[];
      for i=1:length(beginSen)-1
         sentence=[sentence ' ' DScreateSentence(data(beginSen(i):beginSen(i+1)-1))];
      end;
      sentence=[sentence ' ' DScreateSentence(data(beginSen(end):end))];
   end
end;
