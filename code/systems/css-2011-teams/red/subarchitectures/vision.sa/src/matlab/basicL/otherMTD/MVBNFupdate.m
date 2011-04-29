function [mC1,mDA1,mFS1]=MVBNFupdate(F,av,mC,mDA,mFS)
%[mC1,mDA1,mFS1]=MVBFupdate(x,av,mC,mDA,mFS)
%MVBF incremental learning - one update step.
%x: input images
%av: attribute values of this image
%mC: current model of AVs
%mDA: current model of detected attributes
%mFS: current feature statistics
%mC1: updated model of AVs
%mDA1: updated model of detected attributes
%mFS1: updated feature statistics

if isempty(av) %no av given => do nothing
   mC1=mC;mDA1=mDA;mFS1=mFS;
else

   if size(av,2)==1 %add 1 weights if not provided
      av=[av ones(size(av,1),1)];
   end;


   N=size(F,2);
   numF=size(F,1);
   numC=length(mC);
   if isempty(mC(1).name), numC=0; end;

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
      AVmeans=repmat(sqrt(mFS.Fvar./Fvar),numC,1).*mFS.Fmeans;
      AVvars=repmat(mFS.Fvar./Fvar,numC,1).*mFS.Fvars;
   end;

   %update varianaces and means
   AVn=mFS.Fns;
   oldAVs=[mC.name];
   for i=1:size(av,1)
      [isOld,idx]=ismember(av(i,1),oldAVs);
      if  isOld %AV exists => update mean and variance
         [AVmeans(idx,:),AVvars(idx,:),AVn(idx)]=updateMV(F,AVmeans(idx,:),AVvars(idx,:),mFS.Fns(idx),av(i,2));
      else %add new AV
         numC=numC+1;
         AVmeans=[AVmeans;F'];
         AVvars=[AVvars;zeros(1,numF)];
         AVn=[AVn;av(i,2)];
         oldAVs=[oldAVs av(i,1)];
      end;
   end;

   mFS1=struct('Fmean',Fmean,'Fvar',Fvar,'Fn',Fn,'Fmeans',AVmeans,'Fvars',AVvars,'Fns',AVn);

   %AV (attribute values)
   
   %select the best F for each AV and save the model (mean,var) for each AV
   mC1=struct('name', zeros(numC,1), 'mean', zeros(numC,1), 'var', zeros(numC,1), 'Fb', zeros(numC,1), 'conf', zeros(numC,1));
   [foo,mins]=min(AVvars');
   for i=1:numC
      mC1(i).name=oldAVs(i);
      mC1(i).Fb=mins(i);
      mC1(i).mean=AVmeans(i,mC1(i).Fb);
      mC1(i).var=AVvars(i,mC1(i).Fb);
      mC1(i).conf=AVn(i);
   end;



   %DA (detected attributes)
   %determine the number of attributes
   atnames=[mC.name];
   usefullF=unique(cat(1,mC.Fb));
   numDA=length(usefullF);
   mDA1=struct('Fb',zeros(numDA,1),'C',zeros(numDA,1));
   for i=1:numDA
      mDA1(i).Fb=usefullF(i);
      mDA1(i).C=atnames(find(cat(1,mC.Fb)==usefullF(i)));
   end
end;


