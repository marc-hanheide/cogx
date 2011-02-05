function [mC,mDA,mFS]=DMVBKNFbatch(F,AV)
%[mC,mDA,mFS]=MVBFbatch(X,AV)
%MVBF batch learning.
%X: input training images
%AV: given AVs
%mC: model of AVs
%mDA: model of detected attributes
%mFS: feature statistics

THR=1;
K=6;

numC=size(AV,1);
namesAV=1:numC;
N=size(F,2);

%normalize features
Fmean=mean(F,2)';
%Fvar=sqrt(var(F,0,2)');
Fvar=var(F,0,2)';
%F=F./repmat(Fvar',1,N);
F=F./repmat(sqrt(Fvar)',1,N);

%calculate varianaces and means
numF=size(F,1);
Fmeans=zeros(numC,numF);
Fvars=zeros(numC,numF);
Fns=zeros(numC,1);
for i=1:numC %for all attribute values
   %idxs=ceil(find(AV==namesAV(i))/size(AV,1));
   idxs=find(AV(i,:)==1);
   Fi=F(:,idxs); %all F values when attribute value was i
   Fmeans(i,:)=mean(Fi,2);
   Fvars(i,:)=var(Fi,0,2);
   Fns(i)=length(idxs);
end;

mFS=struct('Fmean',Fmean,'Fvar',Fvar,'Fn',N,'Fmeans',Fmeans,'Fvars',Fvars,'Fns',Fns);

%AV (attribute values)
%select the best F for each AV and save the model (mean,var) for each AV
mC=struct('name', zeros(numC,K), 'mean', zeros(numC,K), 'var', zeros(numC,K), 'w', zeros(numC,K), 'Fb', zeros(numC,1), 'conf', zeros(numC,1));

for i=1:numC
   Ri=zeros(numF,N);
   for j=1:numF
      for k=1:N
         ri=abs(F(j,k)-Fmeans(i,j))/sqrt(Fvars(i,j));
         Ri(j,k)=ri<THR;
      end
   end
         
   lgt=[AV(i,:)==1];
   Rgt=repmat(lgt,numF,1);
   RR=Ri==Rgt;
   rr=sum(RR,2);
   
   [rrs,maxs]=sort(-rr);
   rrsn=rrs./repmat(rrs(1),numF,1);
   mC(i).name=namesAV(i);
   mC(i).Fb=maxs(1:K);
   mC(i).mean=Fmeans(i,mC(i).Fb);
   mC(i).var=Fvars(i,mC(i).Fb);
   mC(i).w=rrsn(1:K);
   mC(i).conf=Fns(i);
end;   

%DA (detected attributes)
%determine the number of attributes
usefullF=unique(cat(1,mC.Fb));
numDA=length(usefullF);
mDA=struct('Fb',zeros(numDA,1),'C',zeros(numDA,1));
for i=1:numDA
   mDA(i).Fb=usefullF(i);
   mDA(i).C=find(cat(1,mC.Fb)==usefullF(i));
end   
