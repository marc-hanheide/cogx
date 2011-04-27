function rAVqnt=MVBNFrec(F,mC,mFS)
%rAV=MVBFrecAV(F,mC,THRs);
%Recognize attribute values
%F: feature values
%mC: models of attribute values
%THRs: thresholds for learning
%rAV: recognized AVs in long unordered format (for each AV answer ANSyes or ANSpy, etc.)
%     Note: only already learned AVs are considered and their numbers (sequence) do not
%     neccesarilly correspond to their names (numbers) in general


%confidence threshold
global ML
if ML>3
cthr=10;
else
cthr=2;   
end
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
%F=F./repmat(sqrt(Fvar)',1,N);

%try to recognize all concepts
rAVqnt=zeros(numC,1+N); %determined attribute values
for j=1:numC
   rAVqnt(j,1)=mC(j).name;
   for i=1:N
      if mC(j).conf>=cthr
         if mC(j).var==0, mC(j).var=1e-9; end;
         %rAVqnt(j,i+1)=abs(F(mC(j).Fb,i)-mC(j).mean)/sqrt(mC(j).var);
         rAVqnt(j,i+1)=2*normcdf(-abs(F(mC(j).Fb,i)-mC(j).mean),0,sqrt(mC(j).var));
      else  %if conf<cthr only one sample has been observed => no model yet
         rAVqnt(j,i+1)=Inf;
         %%%rAVqnt(j,i+1)=-Inf;
      end
   end;
end;

