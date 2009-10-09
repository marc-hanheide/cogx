function X=randm(m,n);

if nargin<2 n=m; end;

X=floor(10*rand(m,n));