function VSnextImg

global axCimgH;
global X currIdx;
global numVar varSiz varCol varNoise;

% currIdx=currIdx+1;
% if currIdx>size(X,4)
%    currIdx=1;
%    [X,AV]=genShapes(numVar,varSiz,varCol,varNoise);
%    %permute the data
%    pidxs=randperm(size(X,4));
%    X=X(:,:,:,pidxs);AV=AV(:,pidxs);
% end;   
% x=X(:,:,:,currIdx);

NUMIMGS=300;
currIdx=ceil(rand*NUMIMGS);
x=readImage(currIdx);

%axes(handles.ax_currImg);
%imagesc(x,'Parent',axCimgH);
imshow(x,'Parent',axCimgH);
set(axCimgH,'Visible','off');

ATinterface;