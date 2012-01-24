function [mC,mCG,mFS,numq]=LMupdate(LM,f,cgt,mC,mCG,mFS)

global THRs;
readConstants;

LAs=zeros(2,6,6);

%new learning modes
LAs(:,:,1)=[2 2 2 2 2 2; 0 0 0 0 0 0];CLs(1)=0; %TD
LAs(:,:,2)=[1 1 2 2 2 2; -2 -2 0 0 0 0];CLs(2)=2; %TS
%LAs(:,:,3)=[1 3 3 0 3 2; 1 -3 -3 0 -3 0];CLs(3)=1; %TA
LAs(:,:,3)=[1 3 3 0 3 2; 1 -3 -3 0 0 0];CLs(3)=1; %TA
LAs(:,:,4)=[1 1 0 0 0 2; 1 1 0 0 0 0];CLs(4)=0; %TU

%old learning modes
LAs(:,:,5)=[2 2 2 2 2 2; 0 0 0 0 0 0];CLs(5)=0; %oTD
LAs(:,:,6)=[1 3 3 0 3 2; 1 0 0 0 0 0];CLs(6)=1; %oTSc
LAs(:,:,7)=[1 1 0 0 3 2; 1 1 0 0 0 0];CLs(7)=1; %oTSl
LAs(:,:,8)=[1 0 0 0 0 0; 1 0 0 0 0 0];CLs(8)=0; %oExc
LAs(:,:,9)=[1 1 0 0 0 0; 1 1 0 0 0 0];CLs(9)=0; %oExc


LA=LAs(:,:,LM);CL=CLs(LM);

cGT=cgt;
cRec=qnt2ql(MTDrec(f,mC,mFS),THRs);
cKnown=lf2sfa(cRec,[ANSyes ANSno ANSpy ANSpn ANSdk]); %already known concepts
cNew=cGT(~ismember(cGT,cKnown));
cAll=[cRec;cNew repmat(-2,size(cNew,1),1)];

toUpd=[];
toUnl=[];
numUpd=0;
numUnl=0;
numInf=0;
numAns=0;
numIgn=0;
numLst=0;
numTfa=0;

numCall=size(cAll,1);
for i=1:numCall
   c=cAll(i,1);
   cigt=ismember(cAll(i,1),cGT);
   lai=LA(2-cigt,answ2idx(cAll(i,2)));
   
   if lai>0 %upd
      toUpd=[toUpd;c];
      numUpd=numUpd+1;
   elseif lai<0 %unl
      toUnl=[toUnl;c];
      numUnl=numUnl+1;
   end
   
   if abs(lai)==2 %T
      numInf=numInf+1;
   end;
   
   if abs(lai)==3 %A
      numAns=numAns+1;
   end;
   
end

if ~isempty(toUpd)
   [mC,mCG,mFS]=MTDupdate(f,toUpd,mC,mCG,mFS);
end;
if ~isempty(toUnl)
   [mC,mCG,mFS]=MTDunlearn(f,toUnl,mC,mCG,mFS);
end;



switch CL
   case 0 %IGN
      numIgn=numIgn+1;
   case 1 %LST
      numLst=numLst+1;
   case 2 %TRD
      numTfa=numTfa+1;
end;

% if CL==2 %TRD
%    numTfa=numTfa+1;
% end;


%numq=numInf+numAns;
cInf=1;
cAns=.25;
cLst=.25;
cTfa=cLst;%2;
numq=cInf*numInf+cAns*numAns+cLst*numLst+cTfa*numTfa;

% numUpd
% numUnl







function idx=answ2idx(answ)
readConstants;
switch answ
   case ANSyes
      idx=1;
   case ANSpy
      idx=2;
   case ANSpn
      idx=3;
   case ANSno
      idx=4;
   case ANSdk
      idx=5;
   case ANSuk
      idx=6;
end

