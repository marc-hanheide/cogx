function rAVql=qnt2qlD(rAVqnt,THRs,CTT)
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
if nargin<3
   %CTT=[1 1;2 1; 3 1;4 1; 5 2;6 2; 7 3;8 3;9 3;10 3];  %AP
   %CTT=[1 1;2 1; 3 2;4 2; 5 3;6 3; 7 4;8 4;9 4;10 5;11 5];  %Relations
%    CTT=[1 1;2 1; 3 1;4 1; 5 2;6 2; 7 3;8 3;9 3;10 3;...
%         11 4;12 4; 13 5;14 5; 15 6;16 6; 17 7;18 7;19 7;20 8;21 8];  %APrel
   %CTT=[1 1;2 1; 3 1;4 1; 5 1;6 1; 7 1;8 1;9 1; 10 2;11 2];  %AP11
   %CTT=[1 1;2 1; 3 1;4 1; 5 1;6 2; 7 2];  %AP7
   CTT=[1 1;2 1; 3 1;4 1; 5 2;6 2];  %AP6
%   CTT=[1 1;2 1; 3 1;4 1; 5 2;6 2;7 3;8 3];  %AP8
end;   

readConstants;

rAVqnt=abs(rAVqnt);



numAV=size(rAVqnt,1);
N=size(rAVqnt,2)-1;
rAVql=zeros(numAV,1+N); %determined attribute values
for j=1:numAV
   rAVql(j,1)=rAVqnt(j,1);
   aicg=CTT(find(CTT(:,2)==CTT(rAVqnt(j,1),2)),1);
   icg=find(ismember(rAVqnt(:,1),aicg));
   for i=1:N
      if ~isinf(rAVqnt(j,i+1))
      %if rAVqnt(j,i+1)>-1
         vals=rAVqnt(icg,i+1);
         if rAVqnt(j,i+1)<max(vals)
            rAVqnt(j,i+1)=0;
         end;   
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
