function rAVqnt=ABrec(F,mC)
%rAV=MVBFrecAV(F,mC,THRs);
%Recognize attribute values
%F: feature values
%mC: models of attribute values
%THRs: thresholds for learning
%rAV: recognized AVs in long unordered format (for each AV answer ANSyes or ANSpy, etc.)
%     Note: only already learned AVs are considered and their numbers (sequence) do not
%     neccesarilly correspond to their names (numbers) in general


%confidence threshold
cthr=6;   


[numF,N]=size(F);
numC=size(mC,2);

%model just initialised
if isempty(mC(1).name) 
   numC=0; 
end;

%try to recognize all concepts
rAVqnt=zeros(numC,1+N); %determined attribute values
for j=1:numC
   rAVqnt(j,1)=mC(j).name;
   for i=1:N
      if mC(j).conf>=cthr
         rAVqnt(j,i+1) = sign(Classify(mC(j).Learners, mC(j).Weights, F(:,i)));
%         rAVqnt(j,i+1) = Classify(mC(j).Learners, mC(j).Weights, F(:,i));
      else  %if conf<cthr only one sample has been observed => no model yet
         rAVqnt(j,i+1)=Inf;
      end
   end;
end;

%rAVqnt=-(rAVqnt-1);
rAVqnt1=[rAVqnt==1];rAVqnt(:,2:end)=rAVqnt1(:,2:end);