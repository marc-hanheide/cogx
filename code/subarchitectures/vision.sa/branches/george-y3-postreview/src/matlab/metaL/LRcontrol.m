function varargout = LRcontrol(varargin)
% LRCONTROL M-file for LRcontrol.fig
%      LRCONTROL, by itself, creates a new LRCONTROL or raises the existing
%      singleton*.
%
%      H = LRCONTROL returns the handle to a new LRCONTROL or the handle to
%      the existing singleton*.
%
%      LRCONTROL('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in LRCONTROL.M with the given input arguments.
%
%      LRCONTROL('Property','Value',...) creates a new LRCONTROL or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before LRcontrol_OpeningFunction gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to LRcontrol_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help LRcontrol

% Last Modified by GUIDE v2.5 19-Oct-2009 23:59:36

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @LRcontrol_OpeningFcn, ...
                   'gui_OutputFcn',  @LRcontrol_OutputFcn, ...
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


% --- Executes just before LRcontrol is made visible.
function LRcontrol_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to LRcontrol (see VARARGIN)

% Choose default command line output for LRcontrol
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes LRcontrol wait for user response (see UIRESUME)
% uiwait(handles.figure1);
global Settings Params
global currMode
set(handles.cbSaveImages,'Value',Settings.SaveImgs);
set(handles.cbASVon,'Value',Settings.ASVon);
set(handles.cbQnt2qlD,'Value',currMode.qnt2qlD);
set(handles.ed_THRyes,'String',num2str(Params.THRs(1)));
set(handles.ed_THRpy,'String',num2str(Params.THRs(2)));
set(handles.ed_THRno,'String',num2str(Params.THRs(3)));




% --- Outputs from this function are returned to the command line.
function varargout = LRcontrol_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in pb_loadAVm.
function pb_loadAVm_Callback(hObject, eventdata, handles)
% hObject    handle to pb_loadAVm (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global Dirs;
[FileName,PathName] = uigetfile('*.mat','Select the file containing AV models',Dirs.models);
if FileName~=0
   fname=[PathName FileName];
   LRloadAVmodels(fname);
end;   




% --- Executes on button press in pb_saveAVm.
function pb_saveAVm_Callback(hObject, eventdata, handles)
% hObject    handle to pb_saveAVm (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global Dirs;
[FileName,PathName] = uiputfile('*.mat','Save file name',[Dirs.models 'mCs.mat']);   
if FileName~=0
   fname=[PathName FileName];
   LRsaveAVmodels(fname);
end;   




% --- Executes on button press in pb_undo.
function pb_undo_Callback(hObject, eventdata, handles)
% hObject    handle to pb_undo (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global Settings
if Settings.ASVon
   if Settings.Params.ASVidx>1
      Settings.Params.ASVidx=Settings.Params.ASVidx-1;
   end   
   asvLoad;
end


% --- Executes on button press in pb_redo.
function pb_redo_Callback(hObject, eventdata, handles)
% hObject    handle to pb_redo (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global Settings
if Settings.ASVon
   Settings.Params.ASVidx=Settings.Params.ASVidx+1;
   asvLoad;
end   


% --- Executes on button press in cbSaveImages.
function cbSaveImages_Callback(hObject, eventdata, handles)
% hObject    handle to cbSaveImages (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of cbSaveImages

global Settings
Settings.SaveImgs=get(handles.cbSaveImages,'Value')



function ed_THRyes_Callback(hObject, eventdata, handles)
% hObject    handle to ed_THRyes (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ed_THRyes as text
%        str2double(get(hObject,'String')) returns contents of ed_THRyes as a double
global Params
thr=get(handles.ed_THRyes,'String');
Params.THRs(1)=str2double(thr);
Params


% --- Executes during object creation, after setting all properties.
function ed_THRyes_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ed_THRyes (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function ed_THRpy_Callback(hObject, eventdata, handles)
% hObject    handle to ed_THRpy (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ed_THRpy as text
%        str2double(get(hObject,'String')) returns contents of ed_THRpy as a double
global Params;
thr=get(handles.ed_THRpy,'String');
Params.THRs(2)=str2double(thr);
Params


% --- Executes during object creation, after setting all properties.
function ed_THRpy_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ed_THRpy (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function ed_THRno_Callback(hObject, eventdata, handles)
% hObject    handle to ed_THRno (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ed_THRno as text
%        str2double(get(hObject,'String')) returns contents of ed_THRno as a double
global Params;
thr=get(handles.ed_THRno,'String');
Params.THRs(3)=str2double(thr);
Params


% --- Executes during object creation, after setting all properties.
function ed_THRno_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ed_THRno (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in cbASVon.
function cbASVon_Callback(hObject, eventdata, handles)
% hObject    handle to cbASVon (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of cbASVon

global Settings
Settings.ASVon=get(handles.cbASVon,'Value')


% --- Executes on button press in pb_firstAsv.
function pb_firstAsv_Callback(hObject, eventdata, handles)
% hObject    handle to pb_firstAsv (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global Settings
if Settings.ASVon
   Settings.Params.ASVidx=1;
   asvLoad;
end   


% --- Executes on button press in pb_lastAsv.
function pb_lastAsv_Callback(hObject, eventdata, handles)
% hObject    handle to pb_lastAsv (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global Settings
if Settings.ASVon
   Settings.Params.ASVidx=-1;
   asvLoad;
end   


% --- Executes on button press in cbQnt2QlD.
function cbQnt2QlD_Callback(hObject, eventdata, handles)
% hObject    handle to cbQnt2QlD (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of cbQnt2QlD



% --- Executes on button press in cbQnt2qlD.
function cbQnt2qlD_Callback(hObject, eventdata, handles)
% hObject    handle to cbQnt2qlD (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of cbQnt2qlD
global currMode
currMode.qnt2qlD=get(handles.cbQnt2qlD,'Value')
