function C=cc2c(CC,param)
%function CC=c2cc(C,CM)
%From new format CC to old ordered long format C

if ~exist('param')
   param='';
end

%quick fix for empty model
numSC=length(CC);
for sc=1:numSC
   if length(CC{sc})<2
      CC{sc}=[0 0];
   end
end


if strcmp(param,'trim')
   numSC=length(CC);
   for sc=1:numSC
      CC{sc}=CC{sc}(1:end-1,:);
   end
end;
C=vertcat(CC{:});

