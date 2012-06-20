function [pcx,pxc,pc]=MKDBFrecP(F,mC)
%[pcx,pxc,pc]=KDBFDrec(F,mC,mFS)
%Recognize attribute values (quantitative)
%Returns probabilities
%F: input feature vectors
%mC: models of concepts
%mFS: feature statistics
%pcx: P(C|x): recognized probabilities for each concept in long unordered format
%pxc: P(x|C)
%pc: P(C)
%     Note: only already learned Cs are considered and their numbers (sequence) do not
%     neccesarilly correspond to their names (numbers) in general

MINCONF=3; %minimum number of previously observed objects of the particular category

%CM=[1:6;1 1 1 1 2 2]'; %concept number -> concept group mapping
CM=[1:10;1 1 1 1 2 2 3 3 3 3]'; %concept number -> concept group mapping
%CM=[1:7;1 1 1 1 2 2 2]'; %concept number -> concept group mapping
%CM=[1:11;1 1 1 1 1 1 1 1 1 2 2]';

if isempty(mC(1).name)
   pcx=0;pxc=0;pc=0;
   pxc=[];pcx=[];pc=[];
   return;
end

numC=size(mC,2);
if isempty(mC(1).name)
   numC=0; 
end;
N=size(F,2);

Nall=sum([mC.conf])/3;

names=[mC.name];
nCG=max(CM(:,2));
sumCG=zeros(nCG,1);
for i=1:nCG
   sumCG(i)=Nall;
end;

pxc=zeros(numC,1+N);
pc=zeros(numC,1+N);
pcx=zeros(numC,1+N);

%pxc, pc
pci=zeros(numC,1);
for j=1:numC
   cj=mC(j).name;
   pxc(j,1)=cj;
   pc(j,1)=cj;
   pci(j)=mC(j).conf/sumCG(CM(find(CM(:,1)==cj),2));
   for i=1:N
      if mC(j).conf>=MINCONF
         rC = executeOperatorIKDE( mC(j).kde, 'input_data', F(:,i), 'evalPdfOnData', 'selectSubDimensions', mC(j).Fb ) ;
         pxc(j,i+1)=rC.evalpdf;
      else  %if conf<MINCONF only a few samples have been observed => no model yet
         pxc(j,i+1)=0;
      end
   end;
end;
pc(:,2:N+1)=repmat(pci,1,N);

sumpxcpc=zeros(nCG,N+1);
for i=1:nCG
   cs=find(CM(:,2)==i);
   ics=find(ismember(names,cs));
   sumpxcpc(i,:)=sum(pxc(ics,:).*pc(ics,:));
end;

%pcx
for j=1:numC
   ccg=CM(find(CM(:,1)==mC(j).name),2);
   pcx(j,1)=mC(j).name;
   for i=1:N
      if mC(j).conf>=MINCONF
         pcx(j,i+1)=pxc(j,i+1)*pc(j,i+1)/(sumpxcpc(CM(find(CM(:,1)==mC(j).name),2),i+1));%+1/1*1/Nall);
      else  %if conf<MINCONF only a few samples has been observed => no model yet
         pcx(j,i+1)=Inf;%0;
      end
   end;
end;




