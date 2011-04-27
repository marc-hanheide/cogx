function [mC1,mCG1,mFS1]=KDBFunlearn(F,C,mC,mCG,mFS)
%[mC1,mCG1,mFS1]=KDBFunlearn(F,C,mC,mCG,mFS)
%KDBF unlearning
%F: input feature vectors
%C: concept labels for these samples (concepts to unlearn)
%mC: current models of concepts
%mCG: current model of detected concept groups
%mFS: current feature statistics
%mC1: updated models of concepts
%mCG1: updated model of detected concept groups
%mFS1: updated feature statistics

ING=2;
MINVAR=1e-1;

numC=length(mC);
[numF,N]=size(F);
if size(C,1)<numC || max(C(:,1))>1
   C=repmat(sf2lf(C,numC),1,N);
end;

mC1=mC;
mCG1=mCG;
mFS1=mFS;

nsbf=1000; %parameter for belFunction
C_sensor=1e-6;% sensor noise

allCs=[mC.name];

for ii=1:numC
   i=find(allCs==ii);
   idxs=find(C(ii,:)==1); %indices of all samples that belong to concept ii
   if ~isempty(idxs) && ~isempty(mC(i).kde)
      disp(['SizeUL: ' num2str(length(idxs)), '  cUL:  ', num2str(ii)]);
      f=F(mC(i).Fb,idxs); %relevant feature vectors
      mC1(i).kde = executeOperatorIKDE( mC(i).kde, 'input_data', f,'unlearn_with_input' ) ;
      mC1(i).belFun=calcBelFun(mC1(i).kde.pdf,nsbf);
      %mC1(i).conf=Fns(i);

      for j=1:length(idxs)
         [mFS1.Fmeans(i,:),mFS1.Fvars(i,:),mFS1.Fns(i)]=updateMV(F(:,idxs(j)),mFS1.Fmeans(i,:),mFS1.Fvars(i,:),mFS1.Fns(i),-1);
      end
   end;
end;


Fmeans=mFS1.Fmeans;
Fvars=mFS1.Fvars;
Fns=mFS1.Fns;
%SELECTION
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
         %                dst=sgHellinger(f1,f2);
         %                dsts(j,i)=dsts(j,i)+dst;
         %                dsts(k,i)=dsts(k,i)+dst;
         dst=sgHellinger(f1,f2);
         dsts(j,i)=min(dsts(j,i),dst);
         dsts(k,i)=min(dsts(k,i),dst);
      end
   end
end
[foo,Fbs]=max(dsts'); %select best features


%   Fbs(5:6)=[1 1];
%Fbs=[1 5 1 6 1 1];

%REINITIALIZE KDEs if neccessary
%select the best F for each C and save the model for each C
for i=1:numC
   oldFb=mC1(i).Fb;
   newFb=Fbs(i);

   clear pdf1;

   if newFb~=oldFb % new best feature
      disp(['SW-UNL: i=' num2str(i) ' C=' num2str(mC1(i).name) ' Fb:' num2str(oldFb) ' -> ' num2str(newFb) ' (comp=' num2str(Fns(i)) ')']);
      
         if Fns(i)>=ING%5
            x_init=sampleFromGaussian(ING,Fmeans(i,newFb),max(Fvars(i,newFb),MINVAR));
            mC1(i).kde= executeOperatorIKDE( [], 'input_data', x_init, 'add_input' );
            mC1(i).belFun=calcBelFun(mC1(i).kde.pdf,nsbf);
         end
%       
%       
%       pdf1.mu=Fmeans(i,newFb);
%       pdf1.covariances=Fvars(i,newFb);
%       pdf1.weights=1;
%       pdf1.components=1;%Fns(i);
% 
%       initializationMethod = 'Silverman'; %'Plugin' ;
%       scaleErrorThreshold = 1/0.7 ; 1/0.7 ;1.5 ;
%       hellErrorGlobal = 0.1 ;% 0.11 / scaleErrorThreshold ;
%       nMaxComponents = 10 ;
%       nMaxComponentsPrior = nMaxComponents ;
%       initByGaussian.mu=Fmeans(i,newFb);
%       initByGaussian.covariances=Fvars(i,newFb);
%       initByGaussian.num_components=Fns(i);
%       pdf1 = updateIKDE( [], [], ...
%          'initialize', 1 ,...
%          'nMaxComponentsPrior', nMaxComponentsPrior,...
%          'scaleErrorThreshold', scaleErrorThreshold,...
%          'hellErrorGlobal', hellErrorGlobal,...
%          'initializationMethod', initializationMethod,...
%          'initByGaussian', initByGaussian);
% 

      %PACK the results
      mC1(i).Fb=newFb;
%      mC1(i).belFun=calcBelFun(mC1(i).kde.pdf,nsbf);
      mC1(i).conf=Fns(i);
   end;

   %DCG (detect concept groups)
   %determine the number of attributes
   cnames=[mC1.name];
   usefullF=unique(cat(1,mC1.Fb));
   numCG=length(usefullF);
   mCG1=struct('Fb',zeros(numCG,1),'C',zeros(numCG,1));
   for i=1:numCG
      mCG1(i).Fb=usefullF(i);
      mCG1(i).C=cnames(find(cat(1,mC1.Fb)==usefullF(i)));
   end
end;





