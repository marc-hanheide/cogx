function rAVql=qnt2ql(rAVqnt,THRs)
%rAV=MVBFrecAV(F,mAV,THRs);
%Recognize attribute values
%F: feature values
%mAV: models of attribute values
%THRs: thresholds for learning
%rAV: recognized AVs in long unordered format (for each AV answer ANSyes or ANSpy, etc.)
%     Note: only already learned AVs are considered and their numbers (sequence) do not
%     neccesarilly correspond to their names (numbers) in general

if nargin<2
   [THRyes,THRpy,THRno]=deal(2,4,5);
else
   [THRyes,THRpy,THRno]=deal(THRs(1),THRs(2),THRs(3));
end

readConstants;

rAVqnt=abs(rAVqnt);

numAV=size(rAVqnt,1);
N=size(rAVqnt,2)-1;
rAVql=zeros(numAV,1+N); %determined attribute values
for j=1:numAV
   rAVql(j,1)=rAVqnt(j,1);
   for i=1:N
      if ~isinf(rAVqnt(j,i+1))
      %if rAVqnt(j,i+1)>-1
         if rAVqnt(j,i+1)>= THRyes
            rAVql(j,i+1)=ANSyes;
         elseif rAVqnt(j,i+1) >= THRpy
            rAVql(j,i+1)=ANSpy;
         elseif rAVqnt(j,i+1)>=THRno
            rAVql(j,i+1)=ANSpn;
         else
            rAVql(j,i+1)=ANSno;
         end;
      else  %if conf==0 only one sample has been observed => no model yet
         rAVql(j,i+1)=ANSdk;
      end
   end;
end;
