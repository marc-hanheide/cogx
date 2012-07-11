function varargout = VStest(varargin)
% VSTEST MATLAB code for VStest.fig
%      VSTEST, by itself, creates a new VSTEST or raises the existing
%      singleton*.
%
%      H = VSTEST returns the handle to a new VSTEST or the handle to
%      the existing singleton*.
%
%      VSTEST('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in VSTEST.M with the given input arguments.
%
%      VSTEST('Property','Value',...) creates a new VSTEST or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before VStest_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to VStest_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help VStest

% Last Modified by GUIDE v2.5 11-Jul-2012 10:56:47

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @VStest_OpeningFcn, ...
                   'gui_OutputFcn',  @VStest_OutputFcn, ...
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


% --- Executes just before VStest is made visible.
function VStest_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to VStest (see VARARGIN)

% Choose default command line output for VStest
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes VStest wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = VStest_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in pb_testCol.
function pb_testCol_Callback(hObject, eventdata, handles)
% hObject    handle to pb_testCol (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
H=str2num(get(handles.ed_V,'String'))
VStestColors(H);

% --- Executes on button press in pbGenColImg.
function pbGenColImg_Callback(hObject, eventdata, handles)
% hObject    handle to pbGenColImg (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
VSgenColImg;




function ed_V_Callback(hObject, eventdata, handles)
% hObject    handle to ed_V (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ed_V as text
%        str2double(get(hObject,'String')) returns contents of ed_V as a double


% --- Executes during object creation, after setting all properties.
function ed_V_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ed_V (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
