function g=ap2gain(ap)

numAP=length(ap);
if numAP>1
   g=zeros(numAP,1);
   for i=1:numAP
      g(i)=ap2gain(ap(i));
   end;
else
   
   global Params
   
   t1=1;
   tY=Params.THRs(1);
   tPy=Params.THRs(2);
   tPn=Params.THRs(3);
   g1=.5;
   gY=1;
   gPy=.5;
   gPn=0;
   
   if ap>=tY
      g=g1+(g1-gY)/(tY-t1)*(t1-ap);
   elseif ap>=tPy
      g=gY-(gY-gPy)/(tY-tPy)*(tY-ap);
   elseif ap>=tPn
      g=gPy-(gPy-gPn)/(tPy-tPn)*(tPy-ap);
   else
      g=0;
   end
   
   
end