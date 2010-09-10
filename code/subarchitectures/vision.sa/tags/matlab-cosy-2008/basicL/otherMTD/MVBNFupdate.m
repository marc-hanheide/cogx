function [mAV1,mDA1,mFS1]=MVBNFupdate(F,av,mAV,mDA,mFS)
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
      AVvars=[];
   else
      %update overall mean and var
      [Fmean,Fvar,Fn]=updateMV(F,mFS.Fmean,mFS.Fvar,mFS.Fn);
      Fvar(Fvar==0)=1e-9;
      %normalize F
      F=F./repmat(sqrt(Fvar)',1,N);
      %adjust alll vars ans means
      AVmeans=repmat(sqrt(mFS.Fvar./Fvar),numAV,1).*mFS.Fmeans;
      AVvars=repmat(mFS.Fvar./Fvar,numAV,1).*mFS.Fvars;
   end;

   %update varianaces and means
   AVn=mFS.Fns;
   oldAVs=[mAV.name];
   for i=1:size(av,1)
      [isOld,idx]=ismember(av(i,1),oldAVs);
      if  isOld %AV exists => update mean and variance
         [AVmeans(idx,:),AVvars(idx,:),AVn(idx)]=updateMV(F,AVmeans(idx,:),AVvars(idx,:),mFS.Fns(idx),av(i,2));
      else %add new AV
         numAV=numAV+1;
         AVmeans=[AVmeans;F'];
         AVvars=[AVvars;zeros(1,numF)];
         AVn=[AVn;av(i,2)];
         oldAVs=[oldAVs av(i,1)];
      end;
   end;

   mFS1=struct('Fmean',Fmean,'Fvar',Fvar,'Fn',Fn,'Fmeans',AVmeans,'Fvars',AVvars,'Fns',AVn);

   %AV (attribute values)
   
   %select the best F for each AV and save the model (mean,var) for each AV
   mAV1=struct('name', zeros(numAV,1), 'mean', zeros(numAV,1), 'var', zeros(numAV,1), 'Fb', zeros(numAV,1), 'conf', zeros(numAV,1));
   [foo,mins]=min(AVvars');
   for i=1:numAV
      mAV1(i).name=oldAVs(i);
      mAV1(i).Fb=mins(i);
      mAV1(i).mean=AVmeans(i,mAV1(i).Fb);
      mAV1(i).var=AVvars(i,mAV1(i).Fb);
      mAV1(i).conf=AVn(i);
   end;



   %DA (detected attributes)
   %determine the number of attributes
   atnames=[mAV.name];
   usefullF=unique(cat(1,mAV.Fb));
   numDA=length(usefullF);
   mDA1=struct('Fb',zeros(numDA,1),'C',zeros(numDA,1));
   for i=1:numDA
      mDA1(i).Fb=usefullF(i);
      mDA1(i).C=atnames(find(cat(1,mAV.Fb)==usefullF(i)));
   end
end;


