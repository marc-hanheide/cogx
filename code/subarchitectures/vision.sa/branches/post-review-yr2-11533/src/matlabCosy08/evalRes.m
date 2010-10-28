function resAV=evalRes(rAV,AVgt)
%resAV=evalResAV(rAV,AV);
%Evaluate results of AV recognition.
%rAV: recognized AVs in a long format
%AVgt: ground truth AVs in a long format
%resAV: results (recognition score, rec. rate (rr1: considering ANSyes and
%ANSno only, and rr2: considering ANSpy and ANSpn as well), rs, rr1, rr2
%for individual AV).

warning off MATLAB:divideByZero

readConstants;
if (max(rAV(:,1))>1)
   rAV=rAV(:,2:end);
end;   
if (max(AVgt(:,1))>1)
   AVgt=AVgt(:,2:end);
end;

[numAV,Nt]=size(AVgt);

r11= rAV==ANSyes&AVgt==ANSyes; %TPc
r12= rAV==ANSyes&AVgt==ANSno;  %FPc
r21= rAV==ANSpy&AVgt==ANSyes;  %TPl
r22= rAV==ANSpy&AVgt==ANSno;   %FPl
r31= rAV==ANSpn&AVgt==ANSyes;  %FNl
r32= rAV==ANSpn&AVgt==ANSno;   %TNl
r41= rAV==ANSno&AVgt==ANSyes;  %FNc
r42= rAV==ANSno&AVgt==ANSno;   %TNc
r51= rAV==ANSdk&AVgt==ANSyes;  
r52= rAV==ANSdk&AVgt==ANSno;

%recognition score
RS=r11-r12+.5*r21-.5*r22-.5*r31+.5*r32-r41+r42;
rs=sum(RS(:));
rss=sum(RS,2)/Nt*100;

%max. recognition score
rsmax=ANSyes*numAV*Nt;

%recognition rate (accuracy) rank1; rrConservative
RR1=r11+r42;
rr1=sum(RR1(:))/numAV/Nt*100; 
rr1s=sum(RR1,2)/Nt*100;

%recognition rate rank2; rrLiberal
RR2=r11+r21+r32+r42;
rr2=sum(RR2(:))/numAV/Nt*100; 
rr2s=sum(RR2,2)/Nt*100;

%TPF=Sensitivity=Recall=TP/(TP+FN)
A=r11;
B=r11+r41;
tpf1=sum(A(:))/sum(B(:));
tpf1s=sum(A,2)./sum(B,2);

A=(r11+r21);
B=r11+r21+r41+r31;
tpf2=sum(A(:))/sum(B(:));
tpf2s=sum(A,2)./sum(B,2);

%TNF=Specifity=TN/(TN+FP)
A=r42;
B=r42+r12;
tnf1=sum(A(:))/sum(B(:));
tnf1s=sum(A,2)./sum(B,2);

A=r42+r32;
B=r42+r32+r12+r22;
tnf2=sum(A(:))/sum(B(:));
tnf2s=sum(A,2)./sum(B,2);

%Precision=TP/(TP+FP)
A=r11;
B=r11+r12;
prec1=sum(A(:))/sum(B(:));
prec1s=sum(A,2)./sum(B,2);

A=r11+r21;
B=r11+r21+r12+r22;
prec2=sum(A(:))/sum(B(:));
prec2s=sum(A,2)./sum(B,2);





%confusion matrix
AVcm1=zeros(numAV,numAV);
AVcm2=zeros(numAV,numAV);
for i=1:Nt
   gtidxs=find(AVgt(:,i)==1);
   ridxs1=find(rAV(:,i)==ANSyes);
   ridxs2=find(rAV(:,i)==ANSyes | rAV(:,i)==ANSpy);
   for j=1:length(gtidxs)
      for k=1:length(ridxs1)
        AVcm1(ridxs1(k),gtidxs(j))=AVcm1(ridxs1(k),gtidxs(j))+1;
      end;
      for k=1:length(ridxs2)
        AVcm2(ridxs2(k),gtidxs(j))=AVcm2(ridxs2(k),gtidxs(j))+1;
      end;
   end;   
end;   


FP1=r12;
FN1=r41;



resAV=struct('rs',rs,'rr1',rr1,'rr2',rr2,'rss',rss,'rr1s',rr1s,'rr2s',rr2s,'rsmax',rsmax,'RS',RS,...
   'tpf1',tpf1,'tpf2',tpf2,'tpf1s',tpf1s,'tpf2s',tpf2s,...
   'tnf1',tnf1,'tnf2',tnf2,'tnf1s',tnf1s,'tnf2s',tnf2s,...
   'prec1',prec1,'prec2',prec2,'prec1s',prec1s,'prec2s',prec2s,...
   'CM1',AVcm1,'CM2',AVcm2,...
   'FP1',FP1,'FN1',FN1);




return


%others

rAVneat=sum(abs(rAV-AVgt),2); %number of erroneus attribute assigments for each attribute value
rAVrr=100-sum(rAVneat)/(Nt*numAV)*100; %attribute recognition rate
rAVrrs=100-rAVneat/Nt*100;

AVcm=zeros(numAV,numAV);
for i=1:Nt
   gtidxs=find(AVgt(:,i)==1);
   ridxs=find(rAV(:,i)==1);
   for j=1:length(gtidxs)
      for k=1:length(ridxs)
        AVcm(ridxs(k),gtidxs(j))=AVcm(ridxs(k),gtidxs(j))+1;
      end;
   end;   
end;   

rAVerr=[(rAV-AVgt)~=0];



resAV=struct('rs',rs,'rr1',rr1,'rr2',rr2,'rr',rAVrr,'rrs',rAVrrs,'cm',AVcm,'rAV',rAV,'rAVerr',rAVerr);
