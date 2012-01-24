function [mC]=MKDBFunlearn(F,C,mC)
%[mC]=MKDBFupdate(F,C,mC)
%MKDBF incremental learning - one update step.
%F: input feature vectors
%C: concept labels for these samples
%mC: current models of concepts


%parameters
   ING=2; %Initial Gaussians
CCT=.1; % 0.5 %CompressionClusterThreshold
global Params
if ~isempty(Params)
   ING=Params.ING;
end;

global currMode
if ~isempty(currMode)
   MDF=Params.MDF;
   CM=Coma.SCC;
end


ING=2;
MINVAR=1e-1;

numC=length(mC);
[numF,N]=size(F);
if size(C,1)<numC || max(C(:,1))>1
   C=repmat(sf2lf(C,numC),1,N);
end;

C_sensor=1e-6;% sensor noise

allCs=[mC.name];

for ii=1:numC
   i=find(allCs==ii);
   idxs=find(C(ii,:)==1); %indices of all samples that belong to concept ii
   if ~isempty(idxs) && ~isempty(mC(i).kde)
      disp(['SizeUL: ' num2str(length(idxs)), '  cUL:  ', num2str(ii)]);
      f=F(:,idxs); %relevant feature vectors
      %save kdes mC f
%      mC(i).kde = executeOperatorIKDE( mC(i).kde, 'input_data', f,'unlearn_with_input' ) ;
      mC(i).kde = executeOperatorIKDE( mC(i).kde, 'input_data', f,'unlearn_with_input','selectSubDimensions',mC(i).Fb ) ;
      %mC1(i).conf=Fns(i);
   end;
end;

%FEATURE SELECTION

%    for i=1:numC
%       if mC(i).name<5
%          mC(i).Fb=[1 2 3];
%       else
%          mC(i).Fb=4:6;
%       end
%    end
%    return
%
%MDF={1,2,3,1:2,2:3,[1 3],1:3,4,5,6,4:5,5:6,[4,6],4:6};
%MDF={[1 3],1:3,4:6};
%   MDF={1:3,4:6};
numMDF=length(MDF);
%CM=[1:10;1 1 1 1 2 2 3 3 3 3]'; %concept number -> concept group mapping
%CM=[1:8;1 1 1 1 2 2 2 2]'; %concept number -> concept group mapping

[Fbs, M_distances]=selectFeatures(mC,CM,MDF);

% dsts=ones(numC,numMDF)*1e10;
% for i=1:numMDF
%    for j=1:numC
%       for k=j+1:numC
%          if mC(j).conf>=ING && mC(k).conf>=ING && ~isempty(mC(j).kde) && ~isempty(mC(k).kde)
%             res = executeOperatorIKDE( mC(j).kde, 'additional_kde', mC(k).kde, 'evalHellingerBetween' , 'selectSubDimensions', MDF{i} ) ;
%             dst=res.distance_hell;
%             dsts(j,i)=min(dsts(j,i),dst);
%             dsts(k,i)=min(dsts(k,i),dst);
%          end
%       end
%    end
% end
% [foo,Fbs]=max(dsts');

for i=1:numC
   oldFb=mC(i).Fb;
   newFb=MDF{Fbs(i)};
   mC(i).Fb=newFb;
   if ~isequal(newFb,oldFb) %new best feature
      %disp(['SW-UPD: i=' num2str(i) ' C=' num2str(mC(i).name) ' Fb:' num2str(oldFb) ' -> ' num2str(newFb) ' (conf=' num2str(mC(i).conf) ')']);
   end
end




