function rAVqnt=NSNFrec(F,mAV)

k=3;

%confidence threshold
cthr=16;   


[numF,N]=size(F);
numAV=size(mAV,2);
Fvar=mAV(1).Fvar;

%model just initialised
if isempty(mAV(1).name) 
   numAV=0; 
   Fvar=ones(1,numF);
end;

%normalise features
Fn=F./repmat(sqrt(Fvar)',1,N);



rAVqnt=zeros(numAV,1+N); %determined attribute values
for j=1:numAV
   rAVqnt(j,1)=mAV(j).name;
   for i=1:N
      if mAV(j).conf>=cthr
         rAVqnt(j,i+1)=sqrt(sum(((Fn(:,i)-mAV(j).mean)'*mAV(j).EV(:,end-k+1:end)).^2./mAV(j).vars(end-k+1:end)));
         %rAVqnt(j,i+1)=sqrt(sum(((Fn(:,i)-mAV(j).mean)'*mAV(j).EV(:,end-k+1:end)).^2./sqrt(mAV(j).vars(end-k+1:end))));

%          k1=find(mAV(j).vars>1e-9,1,'last');
%          %numF=size(F,1);
%          if k1<k, k1=k; end;
%          rAVqnt(j,i+1)=sqrt(sum(((Fn(:,i)-mAV(j).mean)'*mAV(j).EV(:,k1-k+1:k1)).^2./mAV(j).vars(k1-k+1:k1)));
         
      else  %if conf==1 only one sample has been observed => no model yet
         rAVqnt(j,i+1)=Inf;
      end
   end;
end;

