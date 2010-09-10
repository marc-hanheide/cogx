function v=tnormrnd(m,s,t);
%truncated normal distribution
%values can not be smaller or larger for more than t*sigma than mean

nok=1;
while nok
   v=normrnd(m,s);
   if abs(v-m)<=s*t nok=0; end;
end   



