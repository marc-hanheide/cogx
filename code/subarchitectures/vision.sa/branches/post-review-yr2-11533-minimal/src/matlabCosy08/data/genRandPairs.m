function P=genRandPairs(N,Lim)

if nargin<2 
   Lim=[0 640 0 480];
end;

P(1,:)=Lim(1)+rand(1,2*N)*(Lim(2)-Lim(1));
P(2,:)=Lim(3)+rand(1,2*N)*(Lim(4)-Lim(3));