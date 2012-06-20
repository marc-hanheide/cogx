function rAVqnt=NSNFrec(F,mC)

k=3;

%confidence threshold
cthr=16;   


[numF,N]=size(F);
numC=size(mC,2);
Fvar=mC(1).Fvar;

%model just initialised
if isempty(mC(1).name) 
   numC=0; 
   Fvar=ones(1,numF);
end;

%normalise features
Fn=F./repmat(sqrt(Fvar)',1,N);



rAVqnt=zeros(numC,1+N); %determined attribute values
for j=1:numC
   rAVqnt(j,1)=mC(j).name;
   for i=1:N
      if mC(j).conf>=cthr
         rAVqnt(j,i+1)=sqrt(sum(((Fn(:,i)-mC(j).mean)'*mC(j).EV(:,end-k+1:end)).^2./mC(j).vars(end-k+1:end)));
         %rAVqnt(j,i+1)=sqrt(sum(((Fn(:,i)-mC(j).mean)'*mC(j).EV(:,end-k+1:end)).^2./sqrt(mC(j).vars(end-k+1:end))));

%          k1=find(mC(j).vars>1e-9,1,'last');
%          %numF=size(F,1);
%          if k1<k, k1=k; end;
%          rAVqnt(j,i+1)=sqrt(sum(((Fn(:,i)-mC(j).mean)'*mC(j).EV(:,k1-k+1:k1)).^2./mC(j).vars(k1-k+1:k1)));
         
      else  %if conf==1 only one sample has been observed => no model yet
         rAVqnt(j,i+1)=Inf;
      end
   end;
end;

