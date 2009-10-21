function [mAV,cvars]=NSNFbatch(F,AV)

numAV=size(AV,1);
namesAV=1:numAV;
[numF,N]=size(F);

%normalize features
Fvar=var(F,0,2)';
Fvar(Fvar<1e-9)=1e-9;
Fn=F./repmat(sqrt(Fvar'),1,N);

mAV=struct('name', 0, 'Fvar', zeros(1,numF), 'mean', zeros(numF,1), 'EV', zeros(numF,numF), 'vars', zeros(1,numF), 'conf', 0);

%calculate varianaces and means
for i=1:numAV %for all attribute values
   mAV(i).Fvar=Fvar;
   idxs=find(AV(i,:)==1);
   Fi=Fn(:,idxs); %all F values when attribute value was i

%   [mu,U,l]=pca(Fi);
   Ni=size(Fi,2);
   mu=mean(Fi,2);
   Fd=Fi-repmat(mu,1,Ni);
%    C=Fd*Fd';
%    [U D Ut]=svd(C);
%    l=diag(D)'/Ni;
   C=1/Ni*Fd*Fd';
   [U D]=svd(C);
   l=diag(D)';
   
   mAV(i).mean=mu;
   mAV(i).EV=U;
   l(l<1e-6)=1e-6;
   mAV(i).vars=l;
   mAV(i).conf=length(idxs);
   mAV(i).name=namesAV(i);
   
   cvars(:,:,i)=C;
end;

