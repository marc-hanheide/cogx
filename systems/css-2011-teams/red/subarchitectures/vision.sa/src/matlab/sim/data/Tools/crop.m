function [x1,bb1]=crop(x,bb)

[ih,iw]=size(x);

if nargin<2 %crop binary image
   p1=sum(x,1);
   p1i=find(p1>0);
   p2=sum(x,2);
   p2i=find(p2>0);
   bb=[min(p2i) min(p1i); max(p2i) max(p1i)];  %[minY,maxY;minX,maxX]
end;

try
x1=x(bb(1,1):bb(2,1),bb(1,2):bb(2,2),:);
catch 
    fg = 0 ;
    x1 = x ;
    bb= [1,size(x,1),1,size(x,2)] ;
end
    
bb1=bb;