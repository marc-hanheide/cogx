function vsHs=VSstart

global X currIdx;
global numVar varSiz varCol varNoise;
global axCimgH;

global vsHs;

numVar=1;
varSiz=.05;
varCol=.05;
varNoise=.01;

% [X,AV]=genShapes(numVar,varSiz,varCol,varNoise);
% %permute the data
% pidxs=randperm(size(X,4));
% X=X(:,:,:,pidxs);AV=AV(:,pidxs);
% AV=AV(1:6,:);

currIdx=0;

vsHs=VScontrol;

axCimgH=findobj(vsHs, 'Tag','axCurrImg');

f1=figure;
 set(f1,'Visible','off');
VSnextImg;
%close(f1);



