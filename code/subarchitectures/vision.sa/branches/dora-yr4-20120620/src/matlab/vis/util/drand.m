function idxs=drand(selp,allp)
%DRAND  Select random numbers.
%   IDXS = DRAND(SELP,ALLP) randomly selects specified numbers of integers 
%   from specified interval. If ALLP is a number then this interval is 1:ALLP.
%   If ALLP is a vector then this interval is 1:length(ALLP). If SELP>1 then SELP
%   numbers are selected. If SELP<1 then SELP percents of ALLP are selected.
%   The selected integers are stored in an array IDXS.

if length(allp)>1 allp=length(allp); end;   
if selp<1 
   if selp>=.9999 selp=1; end;  %select all
   selp=selp*allp; 
end;

mask=randperm(allp);
idxs=find(mask<=selp);
