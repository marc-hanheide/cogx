function d=esDist(wn,w);
%ESDIST  Distance between points in eigenspace.
%   D = ESDIST(Wn,W) returns the distance between the point Wn and the point W, 
%   if W is a vector. If W is a matrix, D is a row vector where each element 
%   represents the distance from the point Wn to the vector in the corresponding 
%   column of the matrix W.
%
%   See also CREATEES, IS2ES, ES2IS, ESCLOSEST, SHOWES3.

[n,m]=size(w);
for i=1:m 
   d(i)=sqrt(sum((wn-w(:,i)).^2));
end;   
