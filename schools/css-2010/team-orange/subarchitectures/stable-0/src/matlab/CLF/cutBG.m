function x1=cutBG(x,b)

b1=bin(b);
b2=cat(3,b1,b1,b1);
x1=x;
x1(b2==0)=0;