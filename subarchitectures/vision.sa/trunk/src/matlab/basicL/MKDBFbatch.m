function mC=MKDBFbatch(F,C)

CCT=.5 %CompressionClusterThreshold

numC=size(C,1);
namesC=1:numC;

%ESTIMATE KDEs
%select the best F for each C and save the model for each C
mC=struct('name', [], 'kde', [], 'Fb', [], 'conf', []);

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
MDF={1,2,3,1:2,2:3,[1 3],1:3,4,5,6,4:5,5:6,[4,6],4:6};
%MDF={1:3,4:6};
numMDF=length(MDF);

dsts=ones(numC,numMDF)*1e10;
for i=1:numMDF
   for j=1:numC
      for k=j+1:numC
         res = executeOperatorIKDE( mC(j).kde, 'additional_kde', mC(k).kde, 'evalHellingerBetween' , 'selectSubDimensions', MDF{i} ) ;
         dst=res.distance_hell;
         dsts(j,i)=min(dsts(j,i),dst);
         dsts(k,i)=min(dsts(k,i),dst);
      end
   end
   dsts(:,i)=dsts(:,i).^length(MDF{i});
end
[foo,Fbs]=max(dsts');

for i=1:numC
   mC(i).Fb=MDF{Fbs(i)};
end