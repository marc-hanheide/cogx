function CLFstart(flag)

global Settings Coma

global currMode
global mC
global LRguiL LRguiR

Settings.CAST=1;

Coma.avNames={'red';'green';'blue';'yellow';'square';'triangular';'circular'};
Coma.Cnames=['Rd';'Gr';'Bl';'Yl';'Sq';'Tr';'Cr'];
Coma.Fnames=extAPfeatures;
Coma.SCnames=['Color';'Shape'];


currMode=struct('learnMode',2,'THRs',[5 1 .5]/100,'wT',1,'wYes',.75,'wPy',.25,'qnt2qlD',0,'CTT',[]);
Coma.SCC=[1:7;1 1 1 1 2 2 2]';

global Dirs Data
Settings.SaveImgs=0;
Settings.Params.ImgNo=0;
Dirs.images='./imgs/';
%Dirs.cogLearn='/home/user/localsvn/CosyDevVision/trunk/subarchitectures/vision/src/matlab/';
Dirs.cogLearn='./subarchitectures/vision.sa/src/matlab';
Dirs.models = './subarchitectures/vision.sa/src/matlab';

Settings.ASVon=0;
Settings.Params.ASVidx=0;
Dirs.asv=[Dirs.cogLearn 'asv/'];

Dirs.data=[Dirs.cogLearn 'data/'];
Data.numImgs=300;
Data.currImg=1;
Data.imgFormat='.jpg';
Data.imgName='img';
Data.mskName='msk';
Data.numDigit=3;
Data.cgtName='Cgt.mat';

global Params
Params.HEG=.1;
Params.ING=5;
Params.MINCONF=3;
Params.FV=3;


%Reload global variables from CONFIG FILE
%confFile='./config/cogLearn.config';
confFile='./subarchitectures/vision.sa/config/test-vislearner/cogLearn.config';
loadConfig(confFile);

%Initialize process

%[mC,mDA,mFS]=KDBFinit;
mC=MKDBFinit;

[LRguiL LRguiR]=LRvisStart;

if flag==1
   set(LRguiL,'Visible','On');
   set(LRguiR,'Visible','On');
end;

if length(Data.StartupWithModel) > 0
   LRloadAVmodels(Data.StartupWithModel);
end;
disp('CLF Start');

