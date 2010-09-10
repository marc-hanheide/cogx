%cmlSim


global iclsConfig
global processFile dataFile randAll randTrain
global MTD THRs THRst N Nt Nini testNs LM resEER showRes
global numRuns MTHRS
global eval1LM1 evalmLM1 evalmLMm evalmLMmThr
global evalBatch

global pauseNow stopNow

global F C Fnames Cnames

global Dirs

%deafault directories
Dirs.icls='C:/danijels/Matlab/cmLearn/';
Dirs.config=[Dirs.icls 'files/config/'];
Dirs.data=[Dirs.icls 'files/data/'];

%default param values

iclsConfig=[Dirs.config 'icls.cfg'];
   
processFile='';
%dataFile='C:\danijels\Matlab\cmLearn\Sim\data\objects\dataAP500.mat';
dataFile=[Dirs.data 'dataShapes300AP10.mat']
randAll=1;
randTrain=0;

MTD=1;
THRs=[10 2 1]/100;
THRst=THRs;
N=100;
Nt=50;
Nini=10;
testNs=[1 5:5:N];
LM=1;
resEER=0;
showRes=1;

numRuns=5;
MTHRS=THRs;

pauseNow=0;
stopNow=0;

eval1LM1='test4';
evalmLM1='test4a';
evalmLMm='test4b';
evalmLMmThr='test4c';
evalBatch='testBatch';


%Intiate data
 loadConfig(iclsConfig);
dataFile
load(dataFile);


cmlSimParam;