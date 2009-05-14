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

% Last Modified by GUIDE v2.5 09-Nov-2007 23:38:35

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

[FileName,PathName] = uigetfile('*.mat','Select the file containing AV models');
if FileName~=0
   fname=[PathName FileName];
   LRloadAVmodels(fname);
end;   




% --- Executes on button press in pb_saveAVm.
function pb_saveAVm_Callback(hObject, eventdata, handles)
% hObject    handle to pb_saveAVm (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

[FileName,PathName] = uiputfile('*.mat','Save file name','mAVs.mat');   
if FileName~=0
   fname=[PathName FileName];
   LRsaveAVmodels(fname);
end;   




% --- Executes on button press in pb_undo.
function pb_undo_Callback(hObject, eventdata, handles)
% hObject    handle to pb_undo (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global ASVidx;
ASVidx=ASVidx-1;
asvLoad;


% --- Executes on button press in pb_redo.
function pb_redo_Callback(hObject, eventdata, handles)
% hObject    handle to pb_redo (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global ASVidx;
ASVidx=ASVidx+1;
asvLoad;
