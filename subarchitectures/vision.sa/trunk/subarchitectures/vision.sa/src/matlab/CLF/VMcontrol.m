function varargout = VMcontrol(varargin)
% VMCONTROL M-file for VMcontrol.fig
%      VMCONTROL, by itself, creates a new VMCONTROL or raises the existing
%      singleton*.
%
%      H = VMCONTROL returns the handle to a new VMCONTROL or the handle to
%      the existing singleton*.
%
%      VMCONTROL('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in VMCONTROL.M with the given input arguments.
%
%      VMCONTROL('Property','Value',...) creates a new VMCONTROL or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before VMcontrol_OpeningFunction gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to VMcontrol_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help VMcontrol

% Last Modified by GUIDE v2.5 04-Sep-2006 14:18:38

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @VMcontrol_OpeningFcn, ...
                   'gui_OutputFcn',  @VMcontrol_OutputFcn, ...
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


% --- Executes just before VMcontrol is made visible.
function VMcontrol_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to VMcontrol (see VARARGIN)

% Choose default command line output for VMcontrol
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes VMcontrol wait for user response (see UIRESUME)
% uiwait(handles.figure1);

global currMode;
set(handles.ed_THRyes,'String',num2str(currMode.THRs(1)));
set(handles.ed_THRpy,'String',num2str(currMode.THRs(2)));
set(handles.ed_THRno,'String',num2str(currMode.THRs(3)));
set(handles.ed_wT,'String',num2str(currMode.wT));
set(handles.ed_wYes,'String',num2str(currMode.wYes));
set(handles.ed_wPy,'String',num2str(currMode.wPy));


% --- Outputs from this function are returned to the command line.
function varargout = VMcontrol_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on selection change in lb_learnMode.
function lb_learnMode_Callback(hObject, eventdata, handles)
% hObject    handle to lb_learnMode (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = get(hObject,'String') returns lb_learnMode contents as cell array
%        contents{get(hObject,'Value')} returns selected item from lb_learnMode

global currMode;
val=get(handles.lb_learnMode,'Value');
currMode.learnMode=val;
beep;currMode


% --- Executes during object creation, after setting all properties.
function lb_learnMode_CreateFcn(hObject, eventdata, handles)
% hObject    handle to lb_learnMode (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function ed_THRyes_Callback(hObject, eventdata, handles)
% hObject    handle to ed_THRyes (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ed_THRyes as text
%        str2double(get(hObject,'String')) returns contents of ed_THRyes as a double
global currMode;
thr=get(handles.ed_THRyes,'String');
currMode.THRs(1)=str2double(thr);
beep;currMode



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
global currMode;
thr=get(handles.ed_THRpy,'String');
currMode.THRs(2)=str2double(thr);
beep;currMode



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
global currMode;
thr=get(handles.ed_THRno,'String');
currMode.THRs(3)=str2double(thr);
beep;currMode


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




function ed_Wt_Callback(hObject, eventdata, handles)
% hObject    handle to ed_Wt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ed_Wt as text
%        str2double(get(hObject,'String')) returns contents of ed_Wt as a double
global currMode;
w=get(handles.ed_wT,'String');
currMode.wT=str2double(w);
beep;currMode


% --- Executes during object creation, after setting all properties.
function ed_Wt_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ed_Wt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function ed_wYes_Callback(hObject, eventdata, handles)
% hObject    handle to ed_wYes (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ed_wYes as text
%        str2double(get(hObject,'String')) returns contents of ed_wYes as a double
global currMode;
w=get(handles.ed_wYes,'String');
currMode.wYes=str2double(w);
beep;currMode


% --- Executes during object creation, after setting all properties.
function ed_wYes_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ed_wYes (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function ed_wPy_Callback(hObject, eventdata, handles)
% hObject    handle to ed_wPy (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ed_wPy as text
%        str2double(get(hObject,'String')) returns contents of ed_wPy as a double
global currMode;
w=get(handles.ed_wPy,'String');
currMode.wPy=str2double(w);
beep;currMode


% --- Executes during object creation, after setting all properties.
function ed_wPy_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ed_wPy (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end





function ed_wT_Callback(hObject, eventdata, handles)
% hObject    handle to ed_wT (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ed_wT as text
%        str2double(get(hObject,'String')) returns contents of ed_wT as a double


% --- Executes during object creation, after setting all properties.
function ed_wT_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ed_wT (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


