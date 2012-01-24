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

SEL=2;

if isempty(av) %no av given => do nothing
   mC1=mC;mDA1=mDA;mFS1=mFS;
elseif size(F,2)>1   %several feature vectors given
   for i=1:size(F,2) %proces one by one
      [mC,mDA,mFS]=MVBFupdate(F(:,i),lf2sf(av(:,i)),mC,mDA,mFS);
   end;
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
%       %normalize F
%       F=F./repmat(sqrt(Fvar)',1,N);
%       %adjust alll vars ans means
       AVmeans=mFS.Fmeans;
       AVvars=mFS.Fvars;
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

      %Selection
   %normalize variances - for comarison of different F
   if SEL==1
      AVnvars=AVvars./repmat(Fvar,numC,1);
      [foo,Fbs]=min(AVnvars');
   else
      dsts=zeros(numC,numF);
%      tic
      for i=1:numF
         for j=1:numC
            for k=j+1:numC
               f1.mu=AVmeans(j,i);
               f1.covariances=AVvars(j,i)+1e-6;
               f1.weights=1;
               f2.mu=AVmeans(k,i);
               f2.covariances=AVvars(k,i)+1e-6;
               f2.weights=1;                              
               %dst=suHellinger(f1,f2);
               dst=sgHellinger(f1,f2);               
               dsts(j,i)=dsts(j,i)+dst;
               dsts(k,i)=dsts(k,i)+dst;
            end
         end
      end
 %     toc
      [foo,Fbs]=max(dsts');
   end

   
   %AV (attribute values)
   %select the best F for each AV and save the model (mean,var) for each AV
   mC1=struct('name', zeros(numC,1), 'mean', zeros(numC,1), 'var', zeros(numC,1), 'Fb', zeros(numC,1), 'conf', zeros(numC,1));
   for i=1:numC
      mC1(i).name=oldAVs(i);
      mC1(i).Fb=Fbs(i);
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


