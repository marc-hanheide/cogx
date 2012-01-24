function g=ap2gain(ap)
%g=ap2gain(ap) - Calculate gains form APs
%ap: a vector containing APs for all concepts of one super concept
%g: a vector containing gains for all concepts of one super concept

%thresholds/parameters of the mapping function
global Params
t1=1;
tY=Params.THRs(1);
tPy=Params.THRs(2);
tPn=Params.THRs(3);
g1=.25;
gY=1;
gPy=.5;
gPn=0;

ap0=1-sum(ap); %AP of unknown model

numAP=length(ap);
g=zeros(numAP,1);
for i=1:numAP    
    if ap(i)>=tY
        g(i)=g1+(g1-gY)/(tY-t1)*(t1-ap(i));
    elseif ap(i)>=tPy
        g(i)=gY-(gY-gPy)/(tY-tPy)*(tY-ap(i));
    elseif ap(i)>=tPn
        g(i)=gPy-(gPy-gPn)/(tPy-tPn)*(tPy-ap(i));
    else
        g(i)=0;
    end
    g(i)=g(i)+ap0;    
end
    
