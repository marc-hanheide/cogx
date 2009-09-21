function varargout = VMintProc(varargin)
% VMINTPROC M-file for VMintProc.fig
%      VMINTPROC, by itself, creates a new VMINTPROC or raises the existing
%      singleton*.
%
%      H = VMINTPROC returns the handle to a new VMINTPROC or the handle to
%      the existing singleton*.
%
%      VMINTPROC('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in VMINTPROC.M with the given input arguments.
%
%      VMINTPROC('Property','Value',...) creates a new VMINTPROC or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before VMintProc_OpeningFunction gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to VMintProc_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help VMintProc

% Last Modified by GUIDE v2.5 05-Sep-2006 14:59:27

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @VMintProc_OpeningFcn, ...
                   'gui_OutputFcn',  @VMintProc_OutputFcn, ...
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


% --- Executes just before VMintProc is made visible.
function VMintProc_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to VMintProc (see VARARGIN)

% Choose default command line output for VMintProc
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes VMintProc wait for user response (see UIRESUME)
% uiwait(handles.figure1);

set(handles.lb_intProc, 'String', cell(0));
set(handles.lb_intProc, 'Value', 1);



% --- Outputs from this function are returned to the command line.
function varargout = VMintProc_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on selection change in lb_intProc.
function lb_intProc_Callback(hObject, eventdata, handles)
% hObject    handle to lb_intProc (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = get(hObject,'String') returns lb_intProc contents as cell array
%        contents{get(hObject,'Value')} returns selected item from lb_intProc


% --- Executes during object creation, after setting all properties.
function lb_intProc_CreateFcn(hObject, eventdata, handles)
% hObject    handle to lb_intProc (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


