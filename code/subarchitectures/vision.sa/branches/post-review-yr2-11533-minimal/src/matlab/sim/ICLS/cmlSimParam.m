function varargout = cmlSimParam(varargin)
% CMLSIMPARAM Run ICLS

% Last Modified by GUIDE v2.5 02-Oct-2009 12:20:13

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @cmlSimParam_OpeningFcn, ...
                   'gui_OutputFcn',  @cmlSimParam_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before cmlSimParam is made visible.
function cmlSimParam_OpeningFcn(hObject, eventdata, handles, varargin)
% Choose default command line output for cmlSimParam
handles.output = hObject;
% Update handles structure
guidata(hObject, handles);
% declare global parameters (read default values)
gvGet(handles);
global PBHs;
PBHs(1)=handles.ax_1LM1;
PBHs(2)=handles.ax_mLM1;
PBHs(3)=handles.ax_mLMm;
PBHs(4)=handles.ax_mLMmThr;



% --- Outputs from this function are returned to the command line.
function varargout = cmlSimParam_OutputFcn(hObject, eventdata, handles) 
% Get default command line output from handles structure
varargout{1} = handles.output;



% --- Executes on button press in cb_randAll.
function cb_randAll_Callback(hObject, eventdata, handles)


% --- Executes on button press in cb_randTrain.
function cb_randTrain_Callback(hObject, eventdata, handles)


% --- Executes on button press in cb_showRes.
function cb_showRes_Callback(hObject, eventdata, handles)



function ed_MTD_Callback(hObject, eventdata, handles)

% --- Executes during object creation, after setting all properties.
function ed_MTD_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function ed_THRs_Callback(hObject, eventdata, handles)

% --- Executes during object creation, after setting all properties.
function ed_THRs_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function ed_THRst_Callback(hObject, eventdata, handles)

% --- Executes during object creation, after setting all properties.
function ed_THRst_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function ed_N_Callback(hObject, eventdata, handles)

% --- Executes during object creation, after setting all properties.
function ed_N_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function ed_Nt_Callback(hObject, eventdata, handles)

% --- Executes during object creation, after setting all properties.
function ed_Nt_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end




% --- Executes on button press in pb_paramsSet.
function pb_paramsSet_Callback(hObject, eventdata, handles)
gvSet(handles);

% --- Executes on button press in pb_ParamsGet.
function pb_ParamsGet_Callback(hObject, eventdata, handles)
gvGet(handles);

% --- Executes on button press in pb_ParamsDisplay.
function pb_ParamsDisplay_Callback(hObject, eventdata, handles)
gvDisplay(handles);




% --- Executes on button press in pb_toolsExec.
function pb_toolsExec_Callback(hObject, eventdata, handles)
runCommand(handles);


% --- Executes on button press in cb_resEER.
function cb_resEER_Callback(hObject, eventdata, handles)




% --- Executes on button press in pb_toolsPause.
function pb_toolsPause_Callback(hObject, eventdata, handles)
global pauseNow
pauseNow=1;

% --- Executes on button press in pb_tolsResume.
function pb_tolsResume_Callback(hObject, eventdata, handles)
global pauseNow stopNow
pauseNow=0;
stopNow=0;

% --- Executes on button press in pb_toolsStop.
function pb_toolsStop_Callback(hObject, eventdata, handles)
global stopNow
stopNow=1;




function ed_command_Callback(hObject, eventdata, handles)
runCommand(handles);


% --- Executes during object creation, after setting all properties.
function ed_command_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end





% --- Executes on selection change in pm_MTD.
function pm_MTD_Callback(hObject, eventdata, handles)
set(handles.ed_MTD,'string',get(handles.pm_MTD,'Value'));

% --- Executes during object creation, after setting all properties.
function pm_MTD_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end




% --- Executes on selection change in pm_THRs.
function pm_THRs_Callback(hObject, eventdata, handles)
idx=get(handles.pm_THRs,'Value');
str=get(handles.pm_THRs,'string');
set(handles.ed_THRs,'string',cell2mat(str(idx,:)));



% --- Executes during object creation, after setting all properties.
function pm_THRs_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end




% --- Executes on selection change in pm_THRst.
function pm_THRst_Callback(hObject, eventdata, handles)
idx=get(handles.pm_THRst,'Value');
str=get(handles.pm_THRst,'string');
set(handles.ed_THRst,'string',cell2mat(str(idx,:)));


% --- Executes during object creation, after setting all properties.
function pm_THRst_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in lb_LM.
function lb_LM_Callback(hObject, eventdata, handles)



% --- Executes during object creation, after setting all properties.
function lb_LM_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function ed_confFile_Callback(hObject, eventdata, handles)


% --- Executes during object creation, after setting all properties.
function ed_confFile_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pb_confLoad.
function pb_confLoad_Callback(hObject, eventdata, handles)

global iclsConfig
global processFile dataFile randAll randTrain
global MTD THRs THRst N Nt Nini testNs LM resEER showRes
global numRuns MTHRS
global eval1LM1 evalmLM1 evalmLMm evalmLMmThr
global pauseNow stopNow
global Dirs

%global Dirs;
%[FileName,PathName] = uigetfile('*.mat','Select the file containing AV models',Dirs.models);
[FileName,PathName] = uigetfile([Dirs.config '*.cfg'],'Select the configuration file.');
if FileName~=0
   fname=[PathName FileName];
   iclsConfig=fname;
   loadConfig(iclsConfig);
   set(handles.ed_confFile,'String',iclsConfig)
   gvGet(handles);
   gvDisplay(handles);
end


% --- Executes on button press in pb_confEdit.
function pb_confEdit_Callback(hObject, eventdata, handles)
global iclsConfig
edit(iclsConfig);


function ed_dataFile_Callback(hObject, eventdata, handles)


% --- Executes during object creation, after setting all properties.
function ed_dataFile_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pb_dataLoad.
function pb_dataLoad_Callback(hObject, eventdata, handles)

global dataFile F C Fnames Cnames
global Dirs;
%[FileName,PathName] = uigetfile('*.mat','Select the file containing AV models',Dirs.models);
[FileName,PathName] = uigetfile([Dirs.data '*.mat'],'Select the file containing data');
if FileName~=0
   fname=[PathName FileName];
   dataFile=fname;
   load(fname);
   set(handles.ed_dataFile,'String',fname);
   dispDataInfo(handles);
end;   


% --- Executes on button press in pb_dataOpen.
function pb_dataOpen_Callback(hObject, eventdata, handles)


% --- Executes on button press in pb_dataProcess.
function pb_dataProcess_Callback(hObject, eventdata, handles)


% --- Executes on button press in pb_dataSave.
function pb_dataSave_Callback(hObject, eventdata, handles)


% --- Executes on button press in pb_dataRandAll.
function pb_dataRandAll_Callback(hObject, eventdata, handles)


% --- Executes on button press in pb_dataPrepare.
function pb_dataPrepare_Callback(hObject, eventdata, handles)
evalin('base','getFCdata');


function ed_Nini_Callback(hObject, eventdata, handles)


% --- Executes during object creation, after setting all properties.
function ed_Nini_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pb_eval1LM1.
function pb_eval1LM1_Callback(hObject, eventdata, handles)
evalLM(1);


% --- Executes on button press in pb_evalmLM1.
function pb_evalmLM1_Callback(hObject, eventdata, handles)
evalLM(2);


% --- Executes on button press in pb_evalmLMm.
function pb_evalmLMm_Callback(hObject, eventdata, handles)
evalLM(3);


% --- Executes on button press in pb_evalmLMmThr.
function pb_evalmLMmThr_Callback(hObject, eventdata, handles)
evalLM(4);


% --- Executes on button press in pb_toolsCloseFigs.
function pb_toolsCloseFigs_Callback(hObject, eventdata, handles)
%set(gcf,'HandleVisibility','off');
closefigs;
%set(gcf,'HandleVisibility','callback');




%***********     N E W    F U N C T I O N S      ***********

function gvGet(handles)

global iclsConfig
global dataFile randAll randTrain
global MTD THRs THRst N Nt Nini testNs LM resEER showRes numRuns 


set(handles.ed_confFile,'string',iclsConfig);

set(handles.ed_dataFile,'string',dataFile);
set(handles.cb_randAll,'value',randAll);
set(handles.cb_randTrain,'value',randTrain);

set(handles.ed_MTD,'string',MTD);
set(handles.ed_THRs,'string',num2str(THRs));
set(handles.ed_THRst,'string',num2str(THRst));
set(handles.ed_N,'string',N);
set(handles.ed_Nt,'string',Nt);
set(handles.ed_Nini,'string',Nini);
set(handles.ed_testNs,'string',num2str(testNs));
set(handles.lb_LM,'Value',LM)
set(handles.cb_showRes,'value',showRes);
set(handles.cb_resEER,'value',resEER);
set(handles.ed_numRuns,'string',numRuns);



function gvSet(handles)

global iclsConfig
global dataFile randAll randTrain
global MTD THRs THRst N Nt Nini testNs LM resEER showRes numRuns


iclsConfig=get(handles.ed_confFile,'string');

dataFile=get(handles.ed_dataFile,'string');
randAll=get(handles.cb_randAll,'value');
randTrain=get(handles.cb_randTrain,'value');

MTD=eval(get(handles.ed_MTD,'string'));
THRs=str2num(get(handles.ed_THRs,'string'));
THRst=eval(['[' get(handles.ed_THRst,'string') ']']);
N=eval(get(handles.ed_N,'string'));
Nt=eval(get(handles.ed_Nt,'string'));
Nini=eval(get(handles.ed_Nini,'string'));
testNs=eval(['[' get(handles.ed_testNs,'string') ']']);
LM=get(handles.lb_LM,'Value');
showRes=get(handles.cb_showRes,'value');
resEER=get(handles.cb_resEER,'value');
numRuns=eval(get(handles.ed_numRuns,'string'));


function gvDisplay(handles);

global iclsConfig
global processFile dataFile randAll randTrain
global MTD THRs THRst N Nt Nini testNs LM resEER showRes numRuns
global eval1LM1 evalmLM1 evalmLMm evalmLMmThr

fprintf('* * * * * * * * * * * * *  Current parameter values:  * * * * * * * * * * * *\n');
fprintf('* Config file: %s\n',iclsConfig);
fprintf('* Data file: %s\n',dataFile);
fprintf('* randAll: %1d    randTrain: %1d\n', randAll, randTrain);
fprintf('* MTD: %d',MTD);
fprintf('    THRs: [');fprintf('%-5g', THRs);fprintf(']');
fprintf('    THRst: [');fprintf('%-5g', THRst);fprintf(']\n');
fprintf('* N: %d    Nt:%d   Nini:%d   resEER: %d   showRes: %d   numRuns: %d\n',...
   N,Nt,Nini,resEER,showRes,numRuns);
fprintf('* LM: [');fprintf('%-2d', LM);fprintf(']\n');
fprintf('* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *\n');


function dispDataInfo(handles)

global dataFile
global F C Fnames Cnames

[numF,Nall]=size(F);
numC=size(C,1);
fnamesStr=reshape([Fnames repmat(' ',numF,1)]',1,numF*(size(Fnames,2)+1));
cnamesStr=reshape([Cnames repmat(' ',numC,1)]',1,numC*(size(Cnames,2)+1));

fprintf('* * * * * * * * * * * * * * * *   Data info:  * * * * * * * * * * * * * * * *\n');
fprintf('* Data file: %s\n',dataFile);
fprintf('* Nall: %d\n', Nall);
fprintf('* numF: %2d    Fnames: %s\n', numF, fnamesStr);
fprintf('* numC: %2d    Cnames: %s\n', numC, cnamesStr);
fprintf('* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *\n');



function evalLM(iLM)
global eval1LM1 evalmLM1 evalmLMm evalmLMmThr evalBatch

switch iLM
   case 0
      script=evalBatch;
   case 1
      script=eval1LM1;
   case 2
      script=evalmLM1;
   case 3
      script=evalmLMm;
   case 4
      script=evalmLMmThr;
end
disp(['***** Running ' script ' *****']);
evalin('base',script);


function runCommand(handles)

command=get(handles.ed_command,'string');
evalin('base',command);
set(handles.ed_command,'string','');


%***********    E N D    N E W    F U N C T I O N S      ***********



function ed_numRuns_Callback(hObject, eventdata, handles)
% hObject    handle to ed_numRuns (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ed_numRuns as text
%        str2double(get(hObject,'String')) returns contents of ed_numRuns as a double


% --- Executes during object creation, after setting all properties.
function ed_numRuns_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ed_numRuns (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function ed_testNs_Callback(hObject, eventdata, handles)
% hObject    handle to ed_testNs (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ed_testNs as text
%        str2double(get(hObject,'String')) returns contents of ed_testNs as a double


% --- Executes during object creation, after setting all properties.
function ed_testNs_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ed_testNs (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pb_dataInfo.
function pb_dataInfo_Callback(hObject, eventdata, handles)
dispDataInfo(handles);



% --- Executes on button press in pb_evalBatch.
function pb_evalBatch_Callback(hObject, eventdata, handles)
% hObject    handle to pb_evalBatch (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

evalLM(0);
