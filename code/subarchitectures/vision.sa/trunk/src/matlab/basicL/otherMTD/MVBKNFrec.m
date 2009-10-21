function rAVqnt=MVBKNFrec(F,mAV,mFS)
%rAV=MVBFrecAV(F,mAV,THRs);
%Recognize attribute values
%F: feature values
%mAV: models of attribute values
%THRs: thresholds for learning
%rAV: recognized AVs in long unordered format (for each AV answer ANSyes or ANSpy, etc.)
%     Note: only already learned AVs are considered and their numbers (sequence) do not
%     neccesarilly correspond to their names (numbers) in general

K=6;

%confidence threshold
cthr=6;   

[numF,N]=size(F);
numAV=size(mAV,2);
Fvar=mFS.Fvar;

%model just initialised
if isempty(mAV(1).name) 
   numAV=0; 
   Fvar=ones(1,numF);
end;

%normalise features
F=F./repmat(sqrt(Fvar)',1,N);

%try to recognize all concepts
rAVqnt=zeros(numAV,1+N); %determined attribute values
for j=1:numAV
   rAVqnt(j,1)=mAV(j).name;
   for i=1:N
      if mAV(j).conf>=cthr
         ravs=0;
         for k=1:K
            if mAV(j).var(k)==0, mAV(j).var(k)=1e-9; end;
%            ravs=ravs + abs(F(mAV(j).Fb(k),i)-mAV(j).mean(k))/sqrt(mAV(j).var(k));
            ravs=ravs + mAV(j).w(k)*(abs(F(mAV(j).Fb(k),i)-mAV(j).mean(k))/sqrt(mAV(j).var(k)));
         end;   
         rAVqnt(j,i+1)=ravs/K;
      else  %if conf<cthr only one sample has been observed => no model yet
         rAVqnt(j,i+1)=Inf;
         %%%rAVqnt(j,i+1)=-Inf;
      end
   end;
end;

