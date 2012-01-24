function mC=MKDBFbatch(F,C)

CCT=.5; % 0.1; %CompressionClusterThreshold
SEL=2;

%CM=[1:6;1 1 1 1 2 2]'; %concept number -> concept group mapping
%CM=[1:10;1 1 1 1 2 2 3 3 3 3]'; %concept number -> concept group mapping
%CM=[1:7;1 1 1 1 2 2 2]'; %concept number -> concept group mapping
%CM=[1:11;1 1 1 1 1 1 1 1 1 2 2]';
%CM=[1:8;1 1 1 1 2 2 2 2]'; %concept number -> concept group mapping



global currMode Coma Params
if ~isempty(currMode)
   MDF=Params.MDF;
   CM=Coma.SCC;
end

numC=size(C,1);
namesC=1:numC;

%ESTIMATE KDEs
%select the best F for each C and save the model for each C
mC=struct('name', [], 'kde', [], 'Fb', [], 'conf', [], 'Nall',[]);

mC(1).Nall = size(F,2) ;
for i=1:numC
   mC(i).name=namesC(i);
   idxs=find(C(i,:)==1);
   Fi=F(:,idxs);
   %construct KDE from data
   %mC(i).kde= executeOperatorIKDE( [], 'input_data', Fi, 'add_input');
   mC(i).kde= executeOperatorIKDE( [], 'input_data', Fi, 'add_input', 'compressionClusterThresh', CCT  );
   mC(i).kde= executeOperatorIKDE( mC(i).kde, 'compress_pdf' );
   mC(i).conf=size(Fi,2);
end;

%FEATURE SELECTION
% for i=1:numC
%    if i<5
%       mC(i).Fb=[1 2 3];
%    else
%       mC(i).Fb=4:6;
%    end
% end
% return

%MDF={1,2,1:2,1:3,4:6};
%MDF={1,2,3,1:2,2:3,[1 3],1:3,4,5,6,4:5,5:6,[4,6],4:6};
%MDF={1:3,4:6};

Fbs=selectFeatures(mC,CM,MDF);

% numMDF=length(MDF);
% 
% dsts=ones(numC,numMDF)*1e10;
% for i=1:numMDF
%    for j=1:numC
%       for k=j+1:numC
%          res = executeOperatorIKDE( mC(j).kde, 'additional_kde', mC(k).kde, 'evalHellingerBetween' , 'selectSubDimensions', MDF{i} ) ;
%          dst=res.distance_hell;
%          dsts(j,i)=min(dsts(j,i),dst);
%          dsts(k,i)=min(dsts(k,i),dst);
%       end
%    end
%    dsts(:,i)=dsts(:,i).^length(MDF{i});
% end
% 
% if SEL==1
%    [foo,Fbs]=max(dsts');
% else
%    %select best feature for concept goroups
%    names=[mC.name];
%    nCG=max(CM(:,2));
%    ICM=zeros(numC,2);
%    for i=1:nCG
%       cs=find(CM(:,2)==i);
%       ics=find(ismember(names,cs));
%       ICM(ics,1)=names(ics);
%       ICM(ics,2)=i;
%    end;
%    
%    dsts1=zeros(nCG,numMDF);
%    for i=1:nCG
%       dsts1(i,:)=sum(dsts(ICM(:,2)==i,:));
%    end
%    [foo,Fbs1]=max(dsts1');
%    
%    for i=1:numC
%       Fbs(i)=Fbs1(ICM(i,2));
%    end
% end

%pack the results
for i=1:numC
   mC(i).Fb=MDF{Fbs(i)};
end