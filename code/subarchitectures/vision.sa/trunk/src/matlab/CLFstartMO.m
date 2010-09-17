function CLFstartMO(opSys)

global Settings

Settings.CAST=0;

%set global variables to default values
% addPaths ;
% mylocalMaggot3 = 'D:\Work\Matlab\IncrementalKDE\Maggot3\demo\' ;
% pthis = pwd ; cd(mylocalMaggot3) ;
% installEntireMaggot3() ;
% cd(pthis) ;


global Coma
Coma.avNames={'red';'green';'blue';'yellow';'square';'triangular';'circular'};
Coma.Cnames=['Rd';'Gr';'Bl';'Yl';'Sq';'Tr';'Cr'];
Coma.Fnames=extAPfeatures;
Coma.SCnames=['Color';'Shape'];
Coma.SCC=[1:7;1 1 1 1 2 2 2]';

global currMode
currMode=struct('learnMode',2,'THRs',[5 1 .5]/100,'wT',1,'wYes',.75,'wPy',.25,'qnt2qlD',0,'CTT',[]);

global Dirs Data 
if nargin==0 || strcmp(opSys,'win')
   %Dirs.cogLearn='D:\Work\Matlab\CogX\DK_code\';
   Dirs.cogLearn=[pwd '/'];
else   
   Dirs.cogLearn='./';
end

Settings.SaveImgs=0;
Settings.Params.ImgNo=0;

Settings.ASVon=0;
Settings.Params.ASVidx=0;

Dirs.data=[Dirs.cogLearn 'Sim/data/objects_newvis/'];
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
confFile='config/cogLearn.config';
loadConfig(confFile);

loadpredefinedorder = 0 ; 
Data.allindexes = [] ;
if loadpredefinedorder == 1
    try
        Data.allindexes = load([Dirs.data,'sequence2.sqc']) ;
    catch
        warning('Images indexes failed to load!') ;
    end
end

Dirs.images=[Dirs.cogLearn 'files/images/']';
Dirs.asv=[Dirs.cogLearn 'files/asv/'];

%GT data
Data.Cgt=[];
fid=fopen([Dirs.data Data.cgtName]);
if fid~=-1
   fclose(fid);
   load([Dirs.data Data.cgtName]);
   Data.Cgt=Cgt;
end;   

%How to display
Settings.Mwindows=0;
Settings.Disp.GL=1;
Settings.Disp.GR=1;
Settings.Disp.TL=1;
Settings.Disp.TR=1;

%HTML display
Dirs.disp=[Dirs.cogLearn 'files/disp/'];
global Disp
fclose('all');
Disp.mL=[Dirs.disp 'mL.png'];
copyfile([Dirs.disp 'mLinit.png'],Disp.mL);
Disp.mR=[Dirs.disp 'mR.png'];
copyfile([Dirs.disp 'mRinit.png'],Disp.mR);
%delete([Dirs.disp 'mlog.html']);
%Disp.mlog=fopen([Dirs.disp 'mlog.html'],'a');
%Disp.mL=fopen([Dirs.disp 'mL.html'],'w');
%Disp.mR=fopen([Dirs.disp 'mR.html'],'w');
%delete(Disp.mL);
%delete(Disp.mR);

%start processes

DSstart;
VMstart;
ATstart;
VSstart;
LRstart;

%show windows if requested
global Figs
if Settings.Mwindows
   set(Figs.LRguiL.main,'Visible','On');
   set(Figs.LRguiR.main,'Visible','On');
end;


