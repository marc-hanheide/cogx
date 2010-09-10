%showScDesc


s=imread('C:\danijels\Matlab\data\cogLearn\relations\relImg141.jpg');
figure;imshow(Sii);

load 'C:\danijels\Matlab\data\cogLearn\relations\relDat141.mat';
Rii=rgt.R;
Pii=[rgt.P1;rgt.p2];

aii=[4 5 9;3 6 7]';
Aii=sf2lf(aii,10);

ca=Pii(1,:);
cb=Pii(2,:);


scDesc=sceneDesc(Aii,Rii,Pii);
%describeScene(Aii,Rii,Pii);

   %dfigure(3,3,'Scene');resizeFigs(gcf,3,3);
   subplot(1,2,1);
   imshow(s);
   text(ca(1)-10,ca(2),'A','Color','black');
   text(cb(1)-10,cb(2),'B','Color','white');
%   title(['Scene nr. ' num2str(ii,'%3d')]);
   subplot(1,2,2);
   cla;
   setAxis(0,1,0,1);
   th=text(0,.5,scDesc);
   axis off;