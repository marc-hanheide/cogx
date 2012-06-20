function [mC,mDA,mFS]=MVBFbatch(F,AV)
%[mC,mDA,mFS]=MVBFbatch(X,AV)
%MVBF batch learning.
%X: input training images
%AV: given AVs
%mC: model of AVs
%mDA: model of detected attributes
%mFS: feature statistics

SEL=2;

numC=size(AV,1);
namesAV=1:numC;
N=size(F,2);

%normalize features
Fmean=mean(F,2)';
%Fvar=sqrt(var(F,0,2)');
Fvar=var(F,0,2)';
%F=F./repmat(Fvar',1,N);
%F=F./repmat(sqrt(Fvar)',1,N);

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

%normalize variances - for comarison of different F
if SEL==1
   Fvar=var(F,0,2)';
   Fnvars=Fvars./repmat(Fvar,numC,1);
   [foo,Fbs]=min(Fnvars');
else
   dsts=zeros(numC,numF);
   for i=1:numF
      for j=1:numC
         for k=j+1:numC
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
mC=struct('name', zeros(numC,1), 'mean', zeros(numC,1), 'var', zeros(numC,1), 'Fb', zeros(numC,1), 'conf', zeros(numC,1));
%[foo,mins]=min(Fvars');
for i=1:numC
   mC(i).name=namesAV(i);
   mC(i).Fb=Fbs(i);
% if i<=9, mC(i).Fb=1; else mC(i).Fb=6; end;  
   mC(i).mean=Fmeans(i,mC(i).Fb);
   mC(i).var=Fvars(i,mC(i).Fb);
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
