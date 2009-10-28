function [Fbs, M_distances]=selectFeatures(mC,CM,MDF)

%parameters
SEL=2;
ING=2; %Initial Gaussians
global Params
if ~isempty(Params)
   ING=Params.ING;
end;

% CM=[1:10;1 1 1 1 2 2 3 3 3 3]'; %concept number -> concept group mapping
%
% %MDF={1,2,3,1:2,2:3,[1 3],1:3,4,5,6,4:5,5:6,[4,6],4:6};
% MDF={1,[1 3],1:3,4:6};
% %MDF={1:3,4:6};
% global currMode
% if ~isempty(currMode)
%    MDF=currMode.MDF;
% end
%
numC=length(mC);

M_distances = zeros(numC,numC,length(MDF)) ;
numMDF=length(MDF);
dsts=ones(numC,numMDF)*1e10;
for i=1:numMDF
   for j=1:numC
      for k=j+1:numC
         if mC(j).conf>=ING && mC(k).conf>=ING && ~isempty(mC(j).kde) && ~isempty(mC(k).kde)            
           res = executeOperatorIKDE( mC(j).kde, 'additional_kde',  mC(k).kde, 'evalHellingerBetween' , 'selectSubDimensions', MDF{i} ) ;
           dst=res.distance_hell;
%             res = executeOperatorIKDE( mC(j).kde, 'additional_kde', mC(k).kde, 'evalLikBetween' , 'selectSubDimensions', MDF{i} ) ;            
%  dst=res.distance_nloglik;
            dsts(j,i)=min(dsts(j,i),dst);
            dsts(k,i)=min(dsts(k,i),dst);
            
            M_distances(j,k,i) = dst.^(20*length(MDF{i})) ;
         end
      end
   end
   %normalize distances
   %dsts(:,i)=dsts(:,i).^(1/length(MDF{i}));
 
   
   dsts(:,i)= dsts(:,i).^(20*length(MDF{i})) ; %fr*abs(dsts(:,i)).^length(10*MDF{i}) ; %*(1/length(MDF{i}))^(1/3); %.^length(MDF{i});
   %dsts(:,i)=dsts(:,i).^(-(1+length(MDF{i})/100));
   %dsts(:,i)=dsts(:,i)/length(MDF{i});
end

if SEL==1
   [foo,Fbs]=max(dsts');
else
   %select best feature for concept goroups
   names=[mC.name];
   nCG=max(CM(names,2));
   ICM=zeros(numC,2);
   for i=1:nCG
      cs=find(CM(:,2)==i);
      ics=find(ismember(names,cs));
      ICM(ics,1)=names(ics);
      ICM(ics,2)=i;
   end;
   
   dsts1=zeros(nCG,numMDF);
   for i=1:nCG
      dsts1(i,:)=sum(dsts(ICM(:,2)==i,:));
%       dsts1(i,:)=min(dsts(ICM(:,2)==i,:));
   end
   [foo,Fbs1]=max(dsts1');
   
   Fbs=zeros(1,numC);
   for i=1:numC
      Fbs(i)=Fbs1(ICM(i,2));
   end
end
