%filter images and gt data
%include only selected concepts and confidences


%cOK=1:numC;
%cOK=[1:9 16 17];
%cOK=[1:4 14 15 16 17];
cOK=[1:4 16 17];

CCF=C.*CF;
imgsOK=find(max(CCF(cOK,:)==1));

imgsOK=find(max(CCF(cOK(1:4),:)==1));
% imgsOK2=find(max(CCF(cOK(5:6),:)==1));
% imgsOK=ismember(imgsOK1,imgsOK2);


Cnames=Cnames(cOK,:);
F=F(:,imgsOK);
numC=length(cOK);
C=C(:,imgsOK);
imgIdx=imgIdx(imgsOK);

C=C(cOK,:);


%save data Cnames Fnames numC numF imgIdx N1 N2 N Fall AP CF F C



% C(5,F(4,:)<.05)=1; %Sm
% C(5,F(4,:)>=.05)=0;
% C(6,F(4,:)<.05)=0;%Lr
% C(6,F(4,:)>=.05)=1;