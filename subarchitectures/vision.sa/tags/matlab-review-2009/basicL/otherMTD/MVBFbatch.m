function [mAV,mDA,mFS]=MVBFbatch(F,AV)
%[mAV,mDA,mFS]=MVBFbatch(X,AV)
%MVBF batch learning.
%X: input training images
%AV: given AVs
%mAV: model of AVs
%mDA: model of detected attributes
%mFS: feature statistics

SEL=2;

numAV=size(AV,1);
namesAV=1:numAV;
N=size(F,2);

%normalize features
Fmean=mean(F,2)';
%Fvar=sqrt(var(F,0,2)');
Fvar=var(F,0,2)';
%F=F./repmat(Fvar',1,N);
%F=F./repmat(sqrt(Fvar)',1,N);

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

%normalize variances - for comarison of different F
if SEL==1
   Fvar=var(F,0,2)';
   Fnvars=Fvars./repmat(Fvar,numAV,1);
   [foo,Fbs]=min(Fnvars');
else
   dsts=zeros(numAV,numF);
   for i=1:numF
      for j=1:numAV
         for k=j+1:numAV
            f1.mu=Fmeans(j,i);
            f1.covariances=Fvars(j,i);
            f1.weights=1;
            f2.mu=Fmeans(k,i);
            f2.covariances=Fvars(k,i);
            f2.weights=1;
            dst=sgHellinger(f1,f2);
            dsts(j,i)=dsts(j,i)+dst;
            dsts(k,i)=dsts(k,i)+dst;
         end
      end
   end
   [foo,Fbs]=max(dsts');
end

%AV (attribute values)
%select the best F for each AV and save the model (mean,var) for each AV
mAV=struct('name', zeros(numAV,1), 'mean', zeros(numAV,1), 'var', zeros(numAV,1), 'Fb', zeros(numAV,1), 'conf', zeros(numAV,1));
%[foo,mins]=min(Fvars');
for i=1:numAV
   mAV(i).name=namesAV(i);
   mAV(i).Fb=Fbs(i);
% if i<=9, mAV(i).Fb=1; else mAV(i).Fb=6; end;  
   mAV(i).mean=Fmeans(i,mAV(i).Fb);
   mAV(i).var=Fvars(i,mAV(i).Fb);
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