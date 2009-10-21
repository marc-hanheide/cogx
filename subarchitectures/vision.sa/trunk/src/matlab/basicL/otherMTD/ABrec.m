function rAVqnt=ABrec(F,mAV)
%rAV=MVBFrecAV(F,mAV,THRs);
%Recognize attribute values
%F: feature values
%mAV: models of attribute values
%THRs: thresholds for learning
%rAV: recognized AVs in long unordered format (for each AV answer ANSyes or ANSpy, etc.)
%     Note: only already learned AVs are considered and their numbers (sequence) do not
%     neccesarilly correspond to their names (numbers) in general


%confidence threshold
cthr=6;   


[numF,N]=size(F);
numAV=size(mAV,2);

%model just initialised
if isempty(mAV(1).name) 
   numAV=0; 
end;

%try to recognize all concepts
rAVqnt=zeros(numAV,1+N); %determined attribute values
for j=1:numAV
   rAVqnt(j,1)=mAV(j).name;
   for i=1:N
      if mAV(j).conf>=cthr
         rAVqnt(j,i+1) = sign(Classify(mAV(j).Learners, mAV(j).Weights, F(:,i)));
%         rAVqnt(j,i+1) = Classify(mAV(j).Learners, mAV(j).Weights, F(:,i));
      else  %if conf<cthr only one sample has been observed => no model yet
         rAVqnt(j,i+1)=Inf;
      end
   end;
end;

%rAVqnt=-(rAVqnt-1);
rAVqnt1=[rAVqnt==1];rAVqnt(:,2:end)=rAVqnt1(:,2:end);