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
Settings.Mwindows=1;

%HTML display
Dirs.disp=[Dirs.cogLearn 'files/disp/'];
global Disp
fclose('all');
%delete([Dirs.disp 'mlog.html']);
%Disp.mlog=fopen([Dirs.disp 'mlog.html'],'a');
%Disp.mL=fopen([Dirs.disp 'mL.html'],'w');
%Disp.mR=fopen([Dirs.disp 'mR.html'],'w');
Disp.mL=[Dirs.disp 'mL.png'];
Disp.mR=[Dirs.disp 'mR.png'];
%delete(Disp.mL);
%delete(Disp.mR);
copyfile([Dirs.disp 'mLinit.png'],Disp.mL); 
copyfile([Dirs.disp 'mRinit.png'],Disp.mR); 

%start processes

dsHs=DSstart;
vmHs=VMstart;
vsHs=VSstart;
[lrcH LRguiL LRguiR]=LRstart;
atH=ATstart;


if Settings.Mwindows
   set(LRguiL,'Visible','On');
   set(LRguiR,'Visible','On');
end;

global clfHs;
%clfHs=[dsHs lrcH vsHs LRguiL LRguiR atH];
%clfHs=[dsHs vmHs vsHs LRguiL LRguiR atH lrcH];
clfHs=[dsHs vmHs vsHs LRguiL LRguiR lrcH];




% Apos=[250 37;%ds
%    133 4;%vm
%    80 35;%vs
%    186 58;% lrc
%    180 4; %lrv
%    153 36; %lre
%   % 137 64;%at
%    133 4];%lrc
% 
% % Apos=[...
% %    298.6000   48.4615   84.2000   41.5385;%ds
% %    182.4000   61.8462   43.2000   28.3077;%vmc
% %    227.6000   47.1538   69.4000   20.8462;%vmip
% %    238.4000   70.7692   58.2000   19.2308;%vs
% %    182.6000   45.0769   38.0000   12.9231;%lrc
% %    286.0000    3.3077   96.4000   38.4615;%lrv
% % %   176.0000    3.3077  108.0000   38.4615;%lre
% %    176.0000    3.3077  90.0000   30.4615;%lre
% %    134.4000   44.9231   45.6000   14.4615];%at
% %    
% %    
% for i=1:7
%    pos=get(clfHs(i),'Position');
%    set(clfHs(i),'Position',[Apos(i,1:2) pos(3:4)]);
% end;   

