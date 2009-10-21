function [mAV1,mDA1,mFS1]=NSNFupdate(F,av,mAV,mDA,mFS)
%[mAV1,mDA1,mFS1]=MVBFupdate(x,av,mAV,mDA,mFS)
%MVBF incremental learning - one update step.
%x: input images
%av: attribute values of this image
%mAV: current model of AVs
%mDA: current model of detected attributes
%mFS: current feature statistics
%mAV1: updated model of AVs
%mDA1: updated model of detected attributes
%mFS1: updated feature statistics

if isempty(av) %no av given => do nothing
   mAV1=mAV;mDA1=mDA;mFS1=mFS;
else

   if size(av,2)==1 %add 1 weights if not provided
      av=[av ones(size(av,1),1)];
   end;


   N=size(F,2);
   numF=size(F,1);
   numAV=length(mAV);
   if isempty(mAV(1).name), numAV=0; end;

   %FS (feature statistics)

   if isempty(mFS.Fmean) %model empty, first update
      mFS.Fvar=ones(1,numF);
      Fmean=F';
      Fvar=ones(1,numF);
      Fn=1;
      AVmeans=[];
      AVcvars=[];
   else
      %update overall mean and var
      [Fmean,Fvar,Fn]=updateMV(F,mFS.Fmean,mFS.Fvar,mFS.Fn);
      Fvar(Fvar==0)=1e-9;
      Fvar(Fvar<1e-9)=1e-9;
      %normalize F
      F=F./repmat(sqrt(Fvar)',1,N);
      %adjust all cvars ans means
      adjFac=sqrt(mFS.Fvar./Fvar);
      AVmeans=repmat(adjFac,numAV,1).*mFS.Fmeans;
      adjMat=adjFac'*adjFac;
      AVcvars=repmat(adjMat,[1,1,numAV]).*mFS.Fcvars;
   end;
   
   
   
   %update particular AV covarianaces and means
   AVn=mFS.Fns;
   oldAVs=[mAV.name];
   for i=1:size(av,1)
      [isOld,idx]=ismember(av(i,1),oldAVs);
      if  isOld %AV exists => update mean and variance
         [AVmeans(idx,:),AVcvars(:,:,idx),AVn(idx)]=updateMC(F,AVmeans(idx,:)',AVcvars(:,:,idx),mFS.Fns(idx));
      else %add new AV
         numAV=numAV+1;
         AVmeans=[AVmeans;F'];
         AVcvars=cat(3,AVcvars,zeros(numF,numF));
         AVn=[AVn;av(i,2)];
         oldAVs=[oldAVs av(i,1)];
      end;
   end;

   mFS1=struct('Fmean',Fmean,'Fvar',Fvar,'Fn',Fn,'Fmeans',AVmeans,'Fcvars',AVcvars,'Fns',AVn);

   %calculate new NSs
   for i=1:numAV %for all attribute values
      mAV1(i).Fvar=Fvar;

      C=AVcvars(:,:,i);   
      [U D]=svd(C);
      l=diag(D)';

      mAV1(i).mean=AVmeans(i,:)';
      mAV1(i).EV=U;
      l(l<1e-6)=1e-6;
      mAV1(i).vars=l;
      mAV1(i).conf=AVn(i);
      mAV1(i).name=oldAVs(i);
   end;

   %foo
   mDA1=mDA;

end;


