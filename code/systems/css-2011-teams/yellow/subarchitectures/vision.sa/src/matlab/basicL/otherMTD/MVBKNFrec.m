function rAVqnt=MVBKNFrec(F,mC,mFS)
%rAV=MVBFrecAV(F,mC,THRs);
%Recognize attribute values
%F: feature values
%mC: models of attribute values
%THRs: thresholds for learning
%rAV: recognized AVs in long unordered format (for each AV answer ANSyes or ANSpy, etc.)
%     Note: only already learned AVs are considered and their numbers (sequence) do not
%     neccesarilly correspond to their names (numbers) in general

K=6;

%confidence threshold
cthr=6;   

[numF,N]=size(F);
numC=size(mC,2);
Fvar=mFS.Fvar;

%model just initialised
if isempty(mC(1).name) 
   numC=0; 
   Fvar=ones(1,numF);
end;

%normalise features
F=F./repmat(sqrt(Fvar)',1,N);

%try to recognize all concepts
rAVqnt=zeros(numC,1+N); %determined attribute values
for j=1:numC
   rAVqnt(j,1)=mC(j).name;
   for i=1:N
      if mC(j).conf>=cthr
         ravs=0;
         for k=1:K
            if mC(j).var(k)==0, mC(j).var(k)=1e-9; end;
%            ravs=ravs + abs(F(mC(j).Fb(k),i)-mC(j).mean(k))/sqrt(mC(j).var(k));
            ravs=ravs + mC(j).w(k)*(abs(F(mC(j).Fb(k),i)-mC(j).mean(k))/sqrt(mC(j).var(k)));
         end;   
         rAVqnt(j,i+1)=ravs/K;
      else  %if conf<cthr only one sample has been observed => no model yet
         rAVqnt(j,i+1)=Inf;
         %%%rAVqnt(j,i+1)=-Inf;
      end
   end;
end;

