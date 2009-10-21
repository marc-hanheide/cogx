function [mC1,mCG1,mFS1]=KDBFupdate(F,C,mC,mCG,mFS)
%[mC1,mCG1,mFS1]=KDBFupdate(F,C,mC,mCG,mFS)
%KDBF incremental learning - one update step.
%F: input feature vectors
%C: concept labels for these samples
%mC: current models of concepts
%mCG: current model of detected concept groups
%mFS: current feature statistics
%mC1: updated models of concepts
%mCG1: updated model of detected concept groups
%mFS1: updated feature statistics


%parameters
COMP=1; %compression on/off
nMaxComponents=15; %limit for compression
SEL=2; %selection method
C_sensor=1e-6;% sensor noise
nsbf=1000; %parameter for belFunction

HEG=.2;%Hellinger Error global
ING=2; %Initial Gaussians
MINVAR=1e-1;
global Params
if ~isempty(Params)
   HEG=Params.HEG;
   ING=Params.ING;
end;

if isempty(C) %no concept given => do nothing
   mC1=mC;mCG1=mCG;mFS1=mFS;
elseif size(F,2)>1   %several feature vectors given
   for i=1:size(F,2) %proces one by one
      [mC,mCG,mFS]=KDBFupdate(F(:,i),lf2sf(C(:,i)),mC,mCG,mFS);
   end;
   mC1=mC;mCG1=mCG;mFS1=mFS;
else
   
   if size(C,2)==1 %add 1 weights if not provided
      C=[C ones(size(C,1),1)];
   end;
   
   numF=size(F,1);
   numC=length(mC);
   if isempty(mC(1).name)
      numC=0;
   end;
   
   %UPDATE GAUSSIANS
   if isempty(mFS.Fmean) %model empty, first update
      %initialize variables
      mFS.Fvar=ones(1,numF);
      Fmean=F';
      Fvar=ones(1,numF);
      Fn=1;
      Fmeans=[];
      Fvars=[];
   else
      %update overall mean and var
      [Fmean,Fvar,Fn]=updateMV(F,mFS.Fmean,mFS.Fvar,mFS.Fn);
      Fvar(Fvar==0)=1e-9;
      Fmeans=mFS.Fmeans;
      Fvars=mFS.Fvars;
   end;
   
   %update varianaces and means
   Fns=mFS.Fns;
   oldCs=[mC.name];
   for i=1:size(C,1)
      [isOld,idx]=ismember(C(i,1),oldCs);
      if  isOld %C exists => update mean and variance
         [Fmeans(idx,:),Fvars(idx,:),Fns(idx)]=updateMV(F,Fmeans(idx,:),Fvars(idx,:),mFS.Fns(idx),C(i,2));
      else %add new C
         numC=numC+1;
         Fmeans=[Fmeans;F'];
         Fvars=[Fvars;zeros(1,numF)];
         Fns=[Fns;C(i,2)];
         oldCs=[oldCs C(i,1)];
      end;
   end;
   
   mFS1=struct('Fmean',Fmean,'Fvar',Fvar,'Fn',Fn,'Fmeans',Fmeans,'Fvars',Fvars,'Fns',Fns);
   
   
   %SELECTION
   if SEL==1 %old simpler selection
      Fnvars=Fvars./repmat(Fvar,numC,1);
      [foo,Fbs]=min(Fnvars');
   elseif SEL==2 %new selection method
      dsts=zeros(numC,numF);
      for i=1:numF
         for j=1:numC
            for k=j+1:numC
               f1.mu=Fmeans(j,i);
               f1.covariances=Fvars(j,i)+C_sensor;
               f1.weights=1;
               f2.mu=Fmeans(k,i);
               f2.covariances=Fvars(k,i)+C_sensor;
               f2.weights=1;
               dst=sgHellinger(f1,f2);
               dsts(j,i)=dsts(j,i)+dst;
               dsts(k,i)=dsts(k,i)+dst;
            end
         end
      end
      [foo,Fbs]=max(dsts'); %select best features
   else %newest selection method
      dsts=ones(numC,numF)*1e10;
      for i=1:numF
         for j=1:numC
            for k=j+1:numC
               f1.mu=Fmeans(j,i);
               f1.covariances=Fvars(j,i)+C_sensor;
               f1.weights=1;
               f2.mu=Fmeans(k,i);
               f2.covariances=Fvars(k,i)+C_sensor;
               f2.weights=1;
               dst=sgHellinger(f1,f2);
               dsts(j,i)=min(dsts(j,i),dst);
               dsts(k,i)=min(dsts(k,i),dst);
            end
         end
      end
      [foo,Fbs]=max(dsts'); %select best features
   end
   
   
   %UPDATE KDEs
   %select the best F for each C and save the model for each C
   mC1=mC;%struct('name', zeros(numC,1), 'mean', zeros(numC,1), 'var', zeros(numC,1), 'Fb', zeros(numC,1), 'conf', zeros(numC,1));
   for i=1:numC
      mC1(i).name=oldCs(i);
      mC1(i).Fb=Fbs(i);
      if i<=length(mC)
         oldFb=mC(i).Fb; %existing concept
      else
         oldFb=0; %new concept
      end;
      newFb=mC1(i).Fb;
      
      if newFb==oldFb %best feature still the same
         
         if ismember(mC1(i).name,C(:,1)) %current feature represents this concept
            
            if Fns(i)==ING
               x_init=sampleFromGaussian(ING,Fmeans(i,newFb),max(Fvars(i,newFb),MINVAR));
               mC1(i).kde= executeOperatorIKDE( [], 'input_data', x_init, 'add_input' );
               mC1(i).belFun=calcBelFun(mC1(i).kde.pdf,nsbf);
            elseif Fns(i)>ING
               %                if mC1(i).name==2
               %                   disp(mC1(i).kde.pdf);
               %                end
               mC1(i).kde = executeOperatorIKDE( mC(i).kde, 'input_data', F(newFb), 'add_input' );
               mC1(i).belFun=calcBelFun(mC1(i).kde.pdf,nsbf);
            end
            
         else %current feature is not of this concept, just copy
            %do nothing
         end
      else %new best feature
         %disp(['SW-UPD: i=' num2str(i) ' C=' num2str(mC1(i).name) ' Fb:' num2str(oldFb) ' -> ' num2str(newFb) ' (comp=' num2str(Fns(i)) ')']);
         if Fns(i)>=ING
            x_init=sampleFromGaussian(ING,Fmeans(i,newFb),max(Fvars(i,newFb),MINVAR));
            mC1(i).kde= executeOperatorIKDE( [], 'input_data', x_init, 'add_input' );
            mC1(i).belFun=calcBelFun(mC1(i).kde.pdf,nsbf);
         end
      end
      
      %PACK the results
      mC1(i).conf=Fns(i);
   end;
   
   
   %DCG (detect concept groups)
   %determine the number of attributes
   cnames=[mC.name];
   usefullF=unique(cat(1,mC.Fb));
   numCG=length(usefullF);
   mCG1=struct('Fb',zeros(numCG,1),'C',zeros(numCG,1));
   for i=1:numCG
      mCG1(i).Fb=usefullF(i);
      mCG1(i).C=cnames(find(cat(1,mC.Fb)==usefullF(i)));
   end
   
   
   
   
end;


