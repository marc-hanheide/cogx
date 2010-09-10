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
   Dirs.cogLearn='D:\Work\Matlab\CogX\DK_code\';
else   
   Dirs.cogLearn='./';
end

global SaveImgs ImgNo
SaveImgs=0;
ImgNo=0;


global ASVon ASVidx
ASVon=0;
ASVidx=0;


Dirs.data=[Dirs.cogLearn 'Sim/data/objects_newvis'];
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

%start processes

dsHs=DSstart;
vmHs=VMstart;
vsHs=VSstart;
[lrcH LRguiL LRguiR]=LRstart;
atH=ATstart;

   set(LRguiL,'Visible','On');
   set(LRguiR,'Visible','On');


global clfHs;
%clfHs=[dsHs lrcH vsHs LRguiL LRguiR atH];
%clfHs=[dsHs vmHs vsHs LRguiL LRguiR atH lrcH];
clfHs=[dsHs vmHs vsHs LRguiL LRguiR lrcH];




Apos=[250 37;%ds
   133 4;%vm
   80 35;%vs
   186 58;% lrc
   180 4; %lrv
   153 36; %lre
  % 137 64;%at
   133 4];%lrc

% Apos=[...
%    298.6000   48.4615   84.2000   41.5385;%ds
%    182.4000   61.8462   43.2000   28.3077;%vmc
%    227.6000   47.1538   69.4000   20.8462;%vmip
%    238.4000   70.7692   58.2000   19.2308;%vs
%    182.6000   45.0769   38.0000   12.9231;%lrc
%    286.0000    3.3077   96.4000   38.4615;%lrv
% %   176.0000    3.3077  108.0000   38.4615;%lre
%    176.0000    3.3077  90.0000   30.4615;%lre
%    134.4000   44.9231   45.6000   14.4615];%at
%    
%    
for i=1:7
   pos=get(clfHs(i),'Position');
   set(clfHs(i),'Position',[Apos(i,1:2) pos(3:4)]);
end;   

%RLcontrol;

% global lrraH lrraxH lrrapH
% lrraH=LRrecAtt;
% lrraxH=findobj(lrraH, 'Tag','axfig');
% lrrapH=findobj(lrraH, 'Tag','axprob');
