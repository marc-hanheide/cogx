function m=rotmatrix(x)
r=x*pi/180;
m=[   0.00000  -sin(r)   cos(r);
  -1.00000   0.00000   0.00000;
   0.00000  -cos(r)  -sin(r)];
