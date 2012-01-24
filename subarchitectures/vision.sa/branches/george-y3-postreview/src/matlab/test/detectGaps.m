function G=detectGaps(rCqnt,mC)

THR_WEAKM=6;
THR_WEAKR=.02;
THR_AMBM=2;

[numC,Nt]=size(rCqnt);

names=rCqnt(:,1);

%Gaps: unknM weakM weakR ambM

unknM=(6-length(mC))/6;

confs=[mC.conf];
weakM=length(find(confs<THR_WEAKM))/6;

weakR=zeros(1,Nt);
ambM=zeros(1,Nt);
idxs1=find(names<=4);
idxs2=find(names>=5);
for i=2:Nt
   ri=rCqnt(idxs1,i);
   ri(isinf(ri))=0;
   [maxc1,idx]=max(ri);
   ri(idx)=-1;
   maxc2=max(ri);
   if maxc1<THR_WEAKR
      weakR(i)=weakR(i)+1;
   end
   if maxc1/maxc2<THR_AMBM
      ambM(i)=ambM(i)+1;
   end
   ri=rCqnt(idxs2,i);
   ri(isinf(ri))=0;
   [maxc1,idx]=max(ri);
   ri(idx)=-1;
   maxc2=max(ri);
   if maxc1<THR_WEAKR
      weakR(i)=weakR(i)+1;
   end
   if maxc1/maxc2<THR_AMBM
      ambM(i)=ambM(i)+1;
   end

end

%G=[unknM; weakM; mean(weakR); mean(ambM)];
G=[unknM; weakM; mean(weakR)/2; mean(ambM)/2];




return





THR_WEAKM=5;
THR_WEAKR=.1;%.02;
THR_AMBM=10;%2;

[numC,Nt]=size(rCqnt);

names=rCqnt(:,1);
idxs=find(names<=4);

%Gaps: unknM weakM weakR ambM

unknM=(6-length(mC))/6;

confs=[mC.conf];
weakM=length(find(confs<THR_WEAKM))/6;

rqs=rCqnt(idxs,:);
maxs=max(rqs);
%NN=mean(maxs)

weakR=zeros(1,Nt);
ambM=zeros(1,Nt);
for i=2:Nt
   ri=rCqnt(idxs,i);
   ri(isinf(ri))=0;
   
   [maxc1,idx]=max(ri);
   %if i==40 disp(maxc1); end;
   ri(idx)=-1;
   maxc2=max(ri);
   if maxc1<THR_WEAKR
      weakR(i)=weakR(i)+1;
   end
   if maxc1/maxc2<THR_AMBM
      ambM(i)=ambM(i)+1;
   end
end

%G=[unknM; weakM; mean(weakR); mean(ambM)];
G=[unknM; weakM; mean(weakR); mean(ambM)];
rCqnt(idxs,:);

%MM=mean(maxs)













G'




return


%function G=detectGaps(rCqnt,mC)

THR_WEAKM=5;
THR_WEAKR=.5;
THR_AMBM=1.1;

[numC,Nt]=size(rCqnt);

%Gaps: unknM weakM weakR ambM

unknM=10-numC;

confs=[mC.conf];
weakM=length(find(confs<THR_WEAKM));

weakR=zeros(1,Nt);
ambM=zeros(1,Nt);
for i=2:Nt
   ri=rCqnt(:,i);
   [maxc1,idx]=max(ri);
   ri(idx)=-1;
   maxc2=max(ri);
   if maxc1<THR_WEAKR
      weakR(i)=weakR(i)+1;
   end
   if maxc1/maxc2<THR_AMBM
      ambM(i)=ambM(i)+1;
   end
end

G=[unknM; weakM; mean(weakR); mean(ambM)];







%OK1:
THR_WEAKM=5;
THR_WEAKR=.1;%.02;
THR_AMBM=10;%2;

[numC,Nt]=size(rCqnt);

names=rCqnt(:,1);
idxs=find(names<=4);

%Gaps: unknM weakM weakR ambM

unknM=(6-length(mC))/6;

confs=[mC.conf];
weakM=length(find(confs<THR_WEAKM))/6;

rqs=rCqnt(idxs,:);
maxs=max(rqs);
%NN=mean(maxs)

weakR=zeros(1,Nt);
ambM=zeros(1,Nt);
for i=2:Nt
   ri=rCqnt(idxs,i);
   ri(isinf(ri))=0;
   
   [maxc1,idx]=max(ri);
   %if i==40 disp(maxc1); end;
   ri(idx)=-1;
   maxc2=max(ri);
   if maxc1<THR_WEAKR
      weakR(i)=weakR(i)+1;
   end
   if maxc1/maxc2<THR_AMBM
      ambM(i)=ambM(i)+1;
   end
end

%G=[unknM; weakM; mean(weakR); mean(ambM)];
G=[unknM; weakM; mean(weakR); mean(ambM)];
