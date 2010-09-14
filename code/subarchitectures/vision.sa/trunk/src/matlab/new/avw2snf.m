function C=avw2snf(avw,CTT)

C=zeros(2,1);
C(CTT(avw(1,1),2))=avw(1,1);
if size(avw,1)>1
   C(CTT(avw(2,1),2))=avw(2,1);
end
