function C=avw2snf(avw,SCC)

C=zeros(2,1);
C(SCC(avw(1,1),2))=avw(1,1);
if size(avw,1)>1
   C(SCC(avw(2,1),2))=avw(2,1);
end
