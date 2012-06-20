function VSnextImg

global axCimgH;
global Data;

Data.currImg=ceil(rand*Data.numImgs);
x=readImage(Data.currImg);

imshow(x,'Parent',axCimgH)
set(axCimgH,'Visible','off');


ATinterface;