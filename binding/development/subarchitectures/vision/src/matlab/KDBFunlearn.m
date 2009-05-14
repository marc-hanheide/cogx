function [mC1,mCG1,mFS1]=KDBFunlearn(F,C,mC,mCG,mFS)
% function [mC,mCG,mFS]=KDBFunlearn(F,C,mC,mCG,mFS)
%KDBF unlearning 
%F: input feature vectors
%C: concept labels for these samples (concepts to unlearn)
%mC: current models of concepts
%mCG: current model of detected concept groups
%mFS: current feature statistics
%mC1: updated models of concepts
%mCG1: updated model of detected concept groups
%mFS1: updated feature statistics


nsbf=1000; %parameter for belFunction

[numF,N]=size(F);
%numC=size(mC,2);
numC=size(C,1);
allCs=[mC.name];

for ii=1:numC
   i=find(allCs==C(ii,1));
   idxs=1;%find(C(i,:)==1);
   if ~isempty(idxs)
      f=F(mC(i).Fb,idxs);
      Covariance = getBW_plugin( mC(i), f(:,1), .5, mFS.Fvars(i,mC(i).Fb),mFS.Fns(i));
      npdf=estimateKDEfromData(f,'Covariance',Covariance);
      if length(npdf.covariances)==1 && npdf.covariances==0
         disp('NPDF covariance is 0!!!!');
         npdf.covariances=1e-6;
      end
      pdf=unlearnAndCompress(mC(i),npdf);
      mC(i).mu=pdf.mu;
      mC(i).covariances=pdf.covariances;
      mC(i).weights=pdf.weights;
      mC(i).belFun=calcBelFun(pdf,nsbf);
   end;
end;

mC1 = mC;
mCG1 = mCG;
mFS1 = mFS;

