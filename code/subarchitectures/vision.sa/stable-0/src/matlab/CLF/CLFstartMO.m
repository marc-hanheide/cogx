function CLFstartMO(opSys)

global CAST
CAST=0;

%set global variables to default values

global avAcronyms avNames
avNames={'red';'green';'blue';'yellow';'square';'triangular';'circular'};
avAcronyms=['Rd';'Gr';'Bl';'Yl';'Sq';'Tr';'Cr'];

global currMode
currMode=struct('learnMode',2,'THRs',[5 1 .5]/100,'wT',1,'wYes',.75,'wPy',.25,'qnt2qlD',0,'CTT',[]);
currMode.CTT=[1:7;1 1 1 1 2 2 2]';

global Dirs Data
if nargin==0 || strcmp(opSys,'win')
   Dirs.cogLearn='C:/danijels/Matlab/cogLearn/';
else   
   Dirs.cogLearn='/home/user/localsvn/CosyDevVision/trunk/subarchitectures/vision/src/matlab/';
end

global SaveImgs ImgNo
SaveImgs=0;
ImgNo=0;
Dirs.images='./imgs/';

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

global Params
Params.HEG=.1;
Params.ING=5;
Params.MINCONF=3;
Params.FV=3;


%Reload global variables from CONFIG FILE
confFile='cogLearn.config';
loadConfig(confFile);

%GT data
Data.Cgt=[];
fid=fopen([Dirs.data Data.cgtName]);
if fid~=-1
   fclose(fid);
   load([Dirs.data Data.cgtName]);
   Data.Cgt=Cgt;
end;   

%start processes

dsHs=DSstart;
vmHs=VMstart;
vsHs=VSstart;
[LRguiL LRguiR]=LRstart;
atH=ATstart;

   set(LRguiL,'Visible','On');
   set(LRguiR,'Visible','On');


global clfHs;
clfHs=[dsHs vmHs vsHs LRguiL LRguiR atH];


Apos=[298 48;%ds
   238 39;%vm
   238 70;%vs
   238 22.5;% lrc
   285 22.5; %lrv
   285 22.5; %lre
   238 5];%at

Apos=[...
   298.6000   48.4615   84.2000   41.5385;%ds
   182.4000   61.8462   43.2000   28.3077;%vmc
   227.6000   47.1538   69.4000   20.8462;%vmip
   238.4000   70.7692   58.2000   19.2308;%vs
   182.6000   45.0769   38.0000   12.9231;%lrc
   286.0000    3.3077   96.4000   38.4615;%lrv
%   176.0000    3.3077  108.0000   38.4615;%lre
   176.0000    3.3077  90.0000   30.4615;%lre
   134.4000   44.9231   45.6000   14.4615];%at
   
   
for i=1:7
   pos=get(clfHs(i),'Position');
   set(clfHs(i),'Position',[Apos(i,1:2) pos(3:4)]);
end;   

%RLcontrol;

% global lrraH lrraxH lrrapH
% lrraH=LRrecAtt;
% lrraxH=findobj(lrraH, 'Tag','axfig');
% lrrapH=findobj(lrraH, 'Tag','axprob');
