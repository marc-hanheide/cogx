function [mC,mCG,mFS]=KDBFbatch(F,C)
%[mC,mCG,mFS]=KDBFbatch(F,C)
%KDBF batch learning
%F: input feature vectors
%C: concept labels for these samples
%mC: estimated models of concepts
%mCG: estimated model of detected concept groups
%mFS: estimated feature statistics

%parameters
COMP=1; %compression: 1=on, 0=off
SEL=3; % selection: 1=var-based, 2=dsts-based
nsbf=1000;

numC=size(C,1);
namesC=1:numC;
N=size(F,2);

%ESTIMATE GAUSSIANS
numF=size(F,1);
Fmeans=zeros(numC,numF);
Fvars=zeros(numC,numF);
Fns=zeros(numC,1);
for i=1:numC %for all concepts
   idxs=find(C(i,:)==1);
   Fi=F(:,idxs); 
   Fmeans(i,:)=mean(Fi,2);
   Fvars(i,:)=var(Fi,0,2);
   Fns(i)=length(idxs);
end;
Fvar=var(F,0,2)';
Fmean=mean(F,2);

%SELECTION
if SEL==1 %old simpler selection
   Fvar=var(F,0,2)';
   Fnvars=Fvars./repmat(Fvar,numC,1);
   [foo,Fbs]=min(Fnvars');
elseif SEL==2 %new selection method
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
            dst=suHellinger(f1,f2);
            dsts(j,i)=dsts(j,i)+dst;
            dsts(k,i)=dsts(k,i)+dst;
         end
      end
   end
   [foo,Fbs]=max(dsts');
   %Fbs(1:4)=1;Fbs(5:7)=7;
elseif SEL==3 %newest selection method
   dsts=ones(numC,numF)*1e10;
   for i=1:numF
      for j=1:numC
         for k=j+1:numC
            f1.mu=Fmeans(j,i);
            f1.covariances=Fvars(j,i);
            f1.weights=1;
            f2.mu=Fmeans(k,i);
            f2.covariances=Fvars(k,i);
            f2.weights=1;
            %dst=suHellinger(f1,f2);
            dst=sgHellinger(f1,f2);
            dsts(j,i)=min(dsts(j,i),dst);
            dsts(k,i)=min(dsts(k,i),dst);
         end
      end
   end
   [foo,Fbs]=max(dsts');
end
mFS=struct('Fmean',Fmean,'Fvar',Fvar,'Fn',N,'Fmeans',Fmeans,'Fvars',Fvars,'Fns',Fns);


%ESTIMATE KDEs
%select the best F for each C and save the model for each C
mC=struct('name', zeros(numC,1), 'mu', zeros(numC,1), 'covariances', zeros(numC,1), 'weights', zeros(numC,1), 'Fb', zeros(numC,1), 'conf', zeros(numC,1));
for i=1:numC
   mC(i).name=namesC(i);
   mC(i).Fb=Fbs(i);
   idxs=find(C(i,:)==1);
   Fi=F(mC(i).Fb,idxs);
   %   pdf = constructKDEfromData( Fi, 'compression', COMP );
   
   %construct KDE from data
   initializationMethod = 'Plugin' ; % 'Silverman'
   reportProgress = 0 ;
   scaleErrorThreshold = 1/0.5; %1/0.7 ; 1/0.7 ;1.5 ;
   hellErrorGlobal = 0.2 ;% 0.11 / scaleErrorThreshold ;
   nMaxComponents = length(Fi) - 1 ;
   nMaxComponentsPrior = nMaxComponents ;
   N_observed = length(Fi) ;
   thresholdMinBandwidth=0.008;
   pdf = updateIKDE( [], Fi, ...
      'initialize', 1 ,...
      'nMaxComponentsPrior', nMaxComponentsPrior,...
      'scaleErrorThreshold', scaleErrorThreshold,...
      'hellErrorGlobal', hellErrorGlobal,...
      'initializationMethod', initializationMethod,...
      'thresholdMinBandwidth', thresholdMinBandwidth);
   
   % compression
   pdf = updateIKDE(pdf, []) ;
%    disp('Compression finished')
   
   pdf2=pdf2mc(pdf);
   mC(i).mu=pdf.mu;
   mC(i).covariances=pdf.covariances;
   mC(i).weights=pdf.weights;
   mC(i).belFun=calcBelFun(mC(i),nsbf);
   mC(i).conf=Fns(i);
   mC(i).components=Fns(i);
   mC(i).params=pdf2.params;
end;

%DCG (detect concept groups)
%determine the number of attributes
usefullF=unique(cat(1,mC.Fb));
numCG=length(usefullF);
mCG=struct('Fb',zeros(numCG,1),'C',zeros(numCG,1));
for i=1:numCG
   mCG(i).Fb=usefullF(i);
   mCG(i).C=find(cat(1,mC.Fb)==usefullF(i));
end

