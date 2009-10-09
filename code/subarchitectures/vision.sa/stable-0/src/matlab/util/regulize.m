function M=regulize(M)
if isnan(det(M))
   disp('NaN!!!!!!!!!!!!!!!!');
   M=0;
end;   
while abs(det(M))<1e-6
   M=M+eye(size(M))*1e-6;
end   