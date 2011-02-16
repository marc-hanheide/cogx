function av=lf2sfa(aav,answ)
%av=av2sf(aav,answ,mAV)
%Long unordered format to short format considering answers
%Returns a list of attribute names (numbers) that are recognised as answ
%aav: recognized AVs in long unordered format (output of MVBFraecAV)
%answ: one or more answers (ANSyes, ANSpy, etc).
%mAV: models of AV

if nargin<2 
   answ=1;
end;   

av=[];j=0;
for i=1:size(aav,1);
   if ismember(aav(i,2),answ)
      j=j+1;
      av(j)=aav(i,1);
   end;
end;