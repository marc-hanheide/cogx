function [pa,pb]=genRel(relNum);

loop=1;
while loop==1
   P=genRandPairs(1);
   R=assignRel(P);
   if R(relNum)==1
      pa=P(:,1);
      pb=P(:,2);
      loop=0;
   end;
end;

