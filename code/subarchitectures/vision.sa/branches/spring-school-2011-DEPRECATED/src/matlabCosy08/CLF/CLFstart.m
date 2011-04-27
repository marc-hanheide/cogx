function CLFstart(flag)

global avAcronyms avNames Fnames
global currState currMode
global mAV mDA mFS
global LRguiL LRguiR
global CAST

CAST=1;

avNames={'red';'green';'blue';'yellow';'square';'triangular';'circular'};
avAcronyms=['Rd';'Gr';'Bl';'Yl';'Sq';'Tr';'Cr'];
Fnames=extAPfeatures;

currMode=struct('learnMode',2,'THRs',[5 1 .5]/100,'wT',1,'wYes',.75,'wPy',.25,'qnt2qlD',0,'CTT',[]);
currMode.CTT=[1:7;1 1 1 1 2 2 2]';

global SaveImgs ImgNo Dirs Data
SaveImgs=0;
ImgNo=0;
Dirs.images='./imgs/';
% Dirs.cogLearn='/home/user/localsvn/CosyDevVision/trunk/subarchitectures/vision/src/matlab/';
Dirs.cogLearn='./subarchitectures/vision.sa/src/matlab';
Dirs.models = './subarchitectures/vision.sa/src/matlab';

global ASVon ASVidx
ASVon=0;
ASVidx=0;
Dirs.asv=[Dirs.cogLearn 'asv/'];

Dirs.data=[Dirs.cogLearn 'data/'];
Data.numImgs=300;
Data.currImg=1;
Data.imgFormat='.jpg';
Data.imgName='img';
Data.mskName='msk';
Data.numDigit=3;
Data.cgtName='Cgt.mat';
Data.StartupWithModel='';

global Params
Params.HEG=.1;
Params.ING=5;
Params.MINCONF=3;
Params.FV=3;


%Reload global variables from CONFIG FILE
confFile='./subarchitectures/vision.sa/config/test-vislearner/cogLearn.config';
loadConfig(confFile);

%Initialize process

[mAV,mDA,mFS]=KDBFinit;

[LRguiL LRguiR]=LRvisStart;

if flag==1
   set(LRguiL,'Visible','On');
   set(LRguiR,'Visible','On');
end;

if length(Data.StartupWithModel) > 0
   LRloadAVmodels(Data.StartupWithModel);
end;

disp('CLF Start');

