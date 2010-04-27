function varargout = LRvis(varargin)
% LRVIS M-file for LRvis.fig
%      LRVIS, by itself, creates a new LRVIS or raises the existing
%      singleton*.
%
%      H = LRVIS returns the handle to a new LRVIS or the handle to
%      the existing singleton*.
%
%      LRVIS('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in LRVIS.M with the given input arguments.
%
%      LRVIS('Property','Value',...) creates a new LRVIS or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before LRvis_OpeningFunction gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to LRvis_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help LRvis

% Last Modified by GUIDE v2.5 04-Sep-2006 23:16:05

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @LRvis_OpeningFcn, ...
                   'gui_OutputFcn',  @LRvis_OutputFcn, ...
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


% --- Executes just before LRvis is made visible.
function LRvis_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to LRvis (see VARARGIN)

% Choose default command line output for LRvis
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes LRvis wait for user response (see UIRESUME)
% uiwait(handles.figure1);

% set(handles.ax_assoc,'PlotBoxAspectRatioMode','auto', ...
%        'DataAspectRatioMode'   ,'auto', ...
%        'CameraViewAngleMode'   ,'auto');
% set(handles.ax_assoc,'Visible','off');
% 
% set(handles.ax_models,'Visible','off');
% 
% global LRvisH
% LRvisH.ax_models=handles.ax_models


% --- Outputs from this function are returned to the command line.
function varargout = LRvis_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;
