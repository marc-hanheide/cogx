function [mAV,mDA,mFS]=DMVBKNFbatch(F,AV)
%[mAV,mDA,mFS]=MVBFbatch(X,AV)
%MVBF batch learning.
%X: input training images
%AV: given AVs
%mAV: model of AVs
%mDA: model of detected attributes
%mFS: feature statistics

THR=1;
K=6;

numAV=size(AV,1);
namesAV=1:numAV;
N=size(F,2);

%normalize features
Fmean=mean(F,2)';
%Fvar=sqrt(var(F,0,2)');
Fvar=var(F,0,2)';
%F=F./repmat(Fvar',1,N);
F=F./repmat(sqrt(Fvar)',1,N);

%calculate varianaces and means
numF=size(F,1);
Fmeans=zeros(numAV,numF);
Fvars=zeros(numAV,numF);
Fns=zeros(numAV,1);
for i=1:numAV %for all attribute values
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
mAV=struct('name', zeros(numAV,K), 'mean', zeros(numAV,K), 'var', zeros(numAV,K), 'w', zeros(numAV,K), 'Fb', zeros(numAV,1), 'conf', zeros(numAV,1));

for i=1:numAV
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
   mAV(i).name=namesAV(i);
   mAV(i).Fb=maxs(1:K);
   mAV(i).mean=Fmeans(i,mAV(i).Fb);
   mAV(i).var=Fvars(i,mAV(i).Fb);
   mAV(i).w=rrsn(1:K);
   mAV(i).conf=Fns(i);
end;   

%DA (detected attributes)
%determine the number of attributes
usefullF=unique(cat(1,mAV.Fb));
numDA=length(usefullF);
mDA=struct('Fb',zeros(numDA,1),'C',zeros(numDA,1));
for i=1:numDA
   mDA(i).Fb=usefullF(i);
   mDA(i).C=find(cat(1,mAV.Fb)==usefullF(i));
end   