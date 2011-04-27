function showProb(pcx,x1)
%show cropped image and probabilities for each attribute value

if nargin==1
   x1=[];
end;   

global lrraxH lrrapH
global Cnames

%cropped object
%imshow(x1,'Parent',lrraxH);

%p(c|x)
bar(pcx(:,2),'b','Parent',lrrapH);
axis(lrrapH,[0 7 0 1]);
Cnames
%set(lrrapH,'XtickLabel',Cnames(pcx(:,1),:));






