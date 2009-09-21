function F=extRelFeatures(P)

numF=5;
Np=size(P,2)/2;
F=zeros(numF,2*Np);
for i=1:Np
   p1=P(:,2*i-1);
   p2=P(:,2*i);
   F(:,2*i-1)=[p1(1); p1(2);  p1(1)-p2(1); p1(2)-p2(2); sqrt((p1(1)-p2(1))^2+(p1(2)-p2(2))^2)];
   p2=P(:,2*i-1);
   p1=P(:,2*i);
   F(:,2*i)=[p1(1); p1(2);  p1(1)-p2(1); p1(2)-p2(2); sqrt((p1(1)-p2(1))^2+(p1(2)-p2(2))^2)];
end;