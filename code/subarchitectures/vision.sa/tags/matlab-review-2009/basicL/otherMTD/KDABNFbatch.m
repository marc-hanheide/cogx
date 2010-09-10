function [mAV,mFS]=KDABNFbatch(F,AV)

numAV=size(AV,1);
namesAV=1:numAV;
N=size(F,2);

%normalize features
Fmean=mean(F,2)';
Fvar=var(F,0,2)';
F=F./repmat(sqrt(Fvar)',1,N);

%calculate varianaces and means
numF=size(F,1);
Fmeans=zeros(numAV,numF);
Fvars=zeros(numAV,numF);
Fns=zeros(numAV,1);
for i=1:numAV %for all attribute values
   idxs=find(AV(i,:)==1);
   Fi=F(:,idxs); %all F values when attribute value was i

   Fmeans(i,:)=mean(Fi,2);
   Fvars(i,:)=var(Fi,0,2);

   Fns(i)=length(idxs);
end;

%mFS=struct('Fmean',Fmean,'Fvar',Fvar,'Fn',N,'Fmeans',Fmeans,'Fvars',Fvars,'Fns',Fns);
mFS=struct('Fvar',Fvar,'Fns',Fns);

%AV (attribute values)
%select the best F for each AV and save the model (mean,var) for each AV
%mAV=struct('name', zeros(numAV,1), 'mu', zeros(numAV,1), 'covariances', zeros(numAV,1), 'weights', zeros(numAV,1), 'Fb', zeros(numAV,1), 'conf', zeros(numAV,1));
[foo,mins]=min(Fvars');
for i=1:numAV
   mAV(i).name=namesAV(i);
   mAV(i).Fb=mins(i);

   idxs=find(AV(i,:)==1);

   for j=1:numF
      Fi=F(j,idxs);
      pdf = constructKDEfromData( Fi, 'compression', 0 );
      mAV(i).mu(j,:)=pdf.mu;
      mAV(i).covariances(:,j)=pdf.covariances;
      mAV(i).weights(j,:)=pdf.weights;
      mAV(i).max(j,:)=pdf.max;
   end
   mAV(i).conf=Fns(i);
end;

% %DA (detected attributes)
% %determine the number of attributes
% usefullF=unique(cat(1,mAV.Fb));
% numDA=length(usefullF);
% mDA=struct('Fb',zeros(numDA,1),'AV',zeros(numDA,1));
% for i=1:numDA
%    mDA(i).Fb=usefullF(i);
%    mDA(i).AV=find(cat(1,mAV.Fb)==usefullF(i));
% end
%
% 1
