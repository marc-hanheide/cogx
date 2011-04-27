function CLFstart(absConfFile)

global Settings Dirs Figs Disp
global mC

if nargin<1
    absConfFile='./config/cogLearn.config';
end

% Default root directory
Dirs.cogLearn=[pwd '/'];

% Load default parameters
%loadDefaultParams;

% Load global variables from CONFIG FILE
confFile = absConfFile
loadConfig(confFile);

% Set the parameters that were not set by loadConfig
loadMissingParams;

%Set initial parameters
Settings.Params.ImgNo=0;
Settings.Params.ASVidx=0;

%Initialize data
disp('Will init data');
if ~Settings.CAST
   initData;
end

%Initialize HTML display
disp('Will copy files');
fclose('all');
copyfile([Dirs.disp 'mGLinit.png'],Disp.mGL);
copyfile([Dirs.disp 'mGRinit.png'],Disp.mGR);
copyfile([Dirs.disp 'mGDinit.png'],Disp.mGD);
copyfile([Dirs.disp 'mTLinit.html'],Disp.mTL);
copyfile([Dirs.disp 'mTRinit.html'],Disp.mTR);
copyfile([Dirs.disp 'mTDinit.html'],Disp.mTD);


%Initialize models
disp('Will ODKDEinit');
mC=ODKDEinit;

%Start GUIs
if ~Settings.CAST
   DSstart;
   VMstart;
   ATstart;
   VSstart;
   LRcontrol;
end

disp('Will LRvisStart');
LRvisStart;
disp('Done LRvisStart');

%Show windows if requested
if Settings.Mwindows
   disp('Will Show Windows');
   set(Figs.LRguiL.main,'Visible','On');
   set(Figs.LRguiR.main,'Visible','On');
end;

%Load models if requested
if ~isempty(Settings.initModel)
   disp('Will Load Models');
   LRloadAVmodels([Dirs.models Settings.initModel]);
end;



% Affordance recognition





disp('CLF Start Done');

function checkInitVal(var, dflt)
% use (at least) the same globals as in loadMissingParams
global Settings Disp Dirs Coma Data Params
global currMode
try
   eval([var ';']);
catch e
   disp(['CLFStart: Setting default for: ' var]);
   eval([var '=' dflt ';']);
end

function loadMissingParams

global Settings Disp Dirs Coma Data Params
global currMode

%default settings
checkInitVal('Settings.CAST', '0');

%How to display
checkInitVal('Settings.Mwindows', '0');
checkInitVal('Settings.Disp.GL', '1');
checkInitVal('Settings.Disp.GR', '1');
checkInitVal('Settings.Disp.TL', '1');
checkInitVal('Settings.Disp.TR', '1');
checkInitVal('Settings.Disp.GD', '1');
checkInitVal('Settings.Disp.TD', '1');
checkInitVal('Settings.Disp.printDpi', '72'); % This affects the size of PNG images
checkInitVal('Settings.SaveImgs', '0');
checkInitVal('Settings.ASVon', '0');
checkInitVal('Settings.initModel', '''''');

%Default directories
checkInitVal('Dirs.cogLearn', '[pwd ''/'']');
checkInitVal('Dirs.images', '[Dirs.cogLearn ''files/images/'']');
checkInitVal('Dirs.models', '[Dirs.cogLearn ''files/models/'']');
checkInitVal('Dirs.asv', '[Dirs.cogLearn ''files/asv/'']');
checkInitVal('Dirs.disp', '[Dirs.cogLearn ''files/disp/'']');

%HTML display files
checkInitVal('Disp.mGL', '[Dirs.disp ''mGL.png'']');
checkInitVal('Disp.mGR', '[Dirs.disp ''mGR.png'']');
checkInitVal('Disp.mGD', '[Dirs.disp ''mGD.png'']');
checkInitVal('Disp.mTL', '[Dirs.disp ''mTL.html'']');
checkInitVal('Disp.mTR', '[Dirs.disp ''mTR.html'']');
checkInitVal('Disp.mTD', '[Dirs.disp ''mTD.html'']');

%Default internal parameters
checkInitVal('Params.THRs', '[.7 .5 .1]');
checkInitVal('Params.FV', '5');
checkInitVal('Params.MINCONF', '3');

%Default conceptual mapping data
checkInitVal('Coma.avNames', '{''red'';''green'';''blue'';''yellow'';''square'';''triangular'';''circular''}');
checkInitVal('Coma.Cnames', '[''Rd'';''Gr'';''Bl'';''Yl'';''Sq'';''Tr'';''Cr'']');
checkInitVal('Coma.Fnames', 'extAPfeatures');
checkInitVal('Coma.SCnames', '[''Color'';''Shape'']');

%Default data
checkInitVal('Dirs.data', '[Dirs.cogLearn ''Sim/data/objects_newvis/'']');
checkInitVal('Data.numImgs', '300');
checkInitVal('Data.currImg', '1');
checkInitVal('Data.imgFormat', '''.jpg''');
checkInitVal('Data.imgName', '''img''');
checkInitVal('Data.mskName', '''msk''');
checkInitVal('Data.numDigit', '3');
checkInitVal('Data.cgtName', '''Cgt.mat''');

%Default mostly obsolete parameters
checkInitVal('Params.HEG', '.1');
checkInitVal('Params.ING', '3');
checkInitVal('Params.SCC', '[1:8; 1 1 1 1 2 2 2 2]');%AP8
checkInitVal('Params.MDF', '{[1 3],1:3,4:6}');
checkInitVal('Params.deterministic ', '0'); 
checkInitVal('currMode', 'struct(''learnMode'',2,''THRs'',[5 1 .5]/100,''wT'',1,''wYes'',.75,''wPy'',.25,''qnt2qlD'',0,''CTT'',[])');
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

%function loadDefaultParams

%global Settings Disp Dirs Coma Data Params

%%default settings
%Settings.CAST=0;

%%How to display
%Settings.Mwindows=0;
%Settings.Disp.GL=1;
%Settings.Disp.GR=1;
%Settings.Disp.TL=1;
%Settings.Disp.TR=1;
%Settings.Disp.GD=1;
%Settings.Disp.TD=1;
%Settings.Disp.printDpi=72; % This affects the size of PNG images
%Settings.SaveImgs=0;
%Settings.ASVon=0;
%Settings.initModel='';

%%Default directories
%Dirs.images=[Dirs.cogLearn 'files/images/'];
%Dirs.models=[Dirs.cogLearn 'files/models/'];
%Dirs.asv=[Dirs.cogLearn 'files/asv/'];
%Dirs.disp=[Dirs.cogLearn 'files/disp/'];

%%HTML display files
%Disp.mGL=[Dirs.disp 'mGL.png'];
%Disp.mGR=[Dirs.disp 'mGR.png'];
%Disp.mGD=[Dirs.disp 'mGD.png'];
%Disp.mTL=[Dirs.disp 'mTL.html'];
%Disp.mTR=[Dirs.disp 'mTR.html'];
%Disp.mTD=[Dirs.disp 'mTD.html'];

%%Default internal parameters
%Params.THRs=[.7 .5 .1];
%Params.FV=5;
%Params.MINCONF=3;

%%Default conceptual mapping data
%Coma.avNames={'red';'green';'blue';'yellow';'square';'triangular';'circular'};
%Coma.Cnames=['Rd';'Gr';'Bl';'Yl';'Sq';'Tr';'Cr'];
%Coma.Fnames=extAPfeatures;
%Coma.SCnames=['Color';'Shape'];

%%Default data
%Dirs.data=[Dirs.cogLearn 'Sim/data/objects_newvis/'];
%Data.numImgs=300;
%Data.currImg=1;
%Data.imgFormat='.jpg';
%Data.imgName='img';
%Data.mskName='msk';
%Data.numDigit=3;
%Data.cgtName='Cgt.mat';

%%Default mostly obsolete parameters
%Params.HEG=.1;
%Params.ING=3;
%Params.SCC=[1:8; 1 1 1 1 2 2 2 2];%AP8
%Params.MDF={[1 3],1:3,4:6};
%Params.deterministic = 0 ; 
%global currMode
%currMode=struct('learnMode',2,'THRs',[5 1 .5]/100,'wT',1,'wYes',.75,'wPy',.25,'qnt2qlD',0,'CTT',[]);


