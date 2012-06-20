function R=assignRel(P)

IH=480;
IW=640;


   numRelGr=5;
   numRel=11;
   numObj=2;

   thrNT=IH/3;%100;
   thrFF=2*IH/3;%200;
   thrOL=IW/3;%100;
   thrOR=2*IW/3;%200;
   thrNR=2*IH/3;%200;
   thrFA=IH/3;%100;

   Np=size(P,2)/2;
   R=zeros(numRel,Np*2);
   
   for k=1:Np
      
      p=P(:,2*k-1:2*k);
      Ri=zeros(numRel,2);
      for i=1:2
         j=rest(1:2,i);
         if (p(1,i)<p(1,j)) Ri(1,i)=1; end;
         if (p(1,i)>p(1,j)) Ri(2,i)=1; end;
         if (p(2,i)<p(2,j)) Ri(3,i)=1; end;
         if (p(2,i)>p(2,j)) Ri(4,i)=1; end;
         if esDist(p(:,i),p(:,j)) < thrNT Ri(5,i)=1; end;
         if esDist(p(:,i),p(:,j)) > thrFF Ri(6,i)=1; end;
         if p(1,i)<=thrOL Ri(7,i)=1; end;
         if p(1,i)>thrOL & p(1,i)<thrOR Ri(8,i)=1; end;
         if p(1,i)>=thrOR Ri(9,i)=1; end;
         if p(2,i)<thrNR Ri(10,i)=1; end;
         if p(2,i)>thrFA Ri(11,i)=1; end;
      end;

      R(:,2*k-1:2*k)=Ri;
   end
