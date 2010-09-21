function CLFstart

global Settings Dirs Figs Disp
global mC

confFile='config/cogLearn.config';
%confFile='./subarchitectures/vision.sa/config/test-vislearner/cogLearn.config';
%confFile='./subarchitectures/vision.sa/config/test-vislearner/cmLearn.config';

%Root directory;
Dirs.cogLearn=[pwd '/'];

%Load default parameters
loadDefaultParams;

%Reload global variables from CONFIG FILE
loadConfig(confFile);

%Set initial parameters
Settings.Params.ImgNo=0;
Settings.Params.ASVidx=0;

%Initialize data
if ~Settings.CAST
   initData;
end

%Initialize HTML display
fclose('all');
copyfile([Dirs.disp 'mGLinit.png'],Disp.mGL);
copyfile([Dirs.disp 'mGRinit.png'],Disp.mGR);
copyfile([Dirs.disp 'mGDinit.png'],Disp.mGD);
copyfile([Dirs.disp 'mTLinit.html'],Disp.mTL);
copyfile([Dirs.disp 'mTRinit.html'],Disp.mTR);
copyfile([Dirs.disp 'mTDinit.html'],Disp.mTD);

%Initialize models
mC=ODKDEinit;

%Start GUIs
if ~Settings.CAST
   DSstart;
   VMstart;
   ATstart;
   VSstart;
   LRcontrol;
end

LRvisStart;

%Show windows if requested
if Settings.Mwindows
   set(Figs.LRguiL.main,'Visible','On');
   set(Figs.LRguiR.main,'Visible','On');
end;

%Load models if requested
if ~isempty(Settings.initModel)
   LRloadAVmodels(Settings.initModel);
end;

disp('CLF Start');




function loadDefaultParams

global Settings Disp Dirs Coma Data Params

%default settings
Settings.CAST=0;

%How to display
Settings.Mwindows=0;
Settings.Disp.GL=1;
Settings.Disp.GR=1;
Settings.Disp.TL=1;
Settings.Disp.TR=1;
Settings.Disp.GD=1;
Settings.Disp.TD=1;
Settings.SaveImgs=0;
Settings.ASVon=0;
Settings.initModel='';

%Default directories
Dirs.images=[Dirs.cogLearn 'files/images/'];
Dirs.models=[Dirs.cogLearn 'files/models/'];
Dirs.asv=[Dirs.cogLearn 'files/asv/'];
Dirs.disp=[Dirs.cogLearn 'files/disp/'];

%HTML display files
Disp.mGL=[Dirs.disp 'mGL.png'];
Disp.mGR=[Dirs.disp 'mGR.png'];
Disp.mGD=[Dirs.disp 'mGD.png'];
Disp.mTL=[Dirs.disp 'mTL.html'];
Disp.mTR=[Dirs.disp 'mTR.html'];
Disp.mTD=[Dirs.disp 'mTD.html'];

%Default internal parameters
Params.THRs=[.7 .5 .1];
Params.FV=5;
Params.MINCONF=3;

%Default conceptual mapping data
Coma.avNames={'red';'green';'blue';'yellow';'square';'triangular';'circular'};
Coma.Cnames=['Rd';'Gr';'Bl';'Yl';'Sq';'Tr';'Cr'];
Coma.Fnames=extAPfeatures;
Coma.SCnames=['Color';'Shape'];

%Default data
Dirs.data=[Dirs.cogLearn 'Sim/data/objects_newvis/'];
Data.numImgs=300;
Data.currImg=1;
Data.imgFormat='.jpg';
Data.imgName='img';
Data.mskName='msk';
Data.numDigit=3;
Data.cgtName='Cgt.mat';

%Default mostly obsolete parameters
Params.HEG=.1;
Params.ING=3;
Params.SCC=[1:8; 1 1 1 1 2 2 2 2];%AP8
Params.MDF={[1 3],1:3,4:6};
Params.deterministic = 0 ; 
global currMode
currMode=struct('learnMode',2,'THRs',[5 1 .5]/100,'wT',1,'wYes',.75,'wPy',.25,'qnt2qlD',0,'CTT',[]);



function initData

global Data Dirs

loadpredefinedorder = 0 ; 
Data.allindexes = [] ;
if loadpredefinedorder == 1
    try
        Data.allindexes = load([Dirs.data,'sequence2.sqc']) ;
    catch
        warning('Images indexes failed to load!') ;
    end
end

%GT data
Data.Cgt=[];
fid=fopen([Dirs.data Data.cgtName]);
if fid~=-1
   fclose(fid);
   load([Dirs.data Data.cgtName]);
   Data.Cgt=Cgt;
end;   
