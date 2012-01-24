function varargout = lrEval(varargin)
% LREVAL M-file for lrEval.fig
%      LREVAL, by itself, creates a new LREVAL or raises the existing
%      singleton*.
%
%      H = LREVAL returns the handle to a new LREVAL or the handle to
%      the existing singleton*.
%
%      LREVAL('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in LREVAL.M with the given input arguments.
%
%      LREVAL('Property','Value',...) creates a new LREVAL or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before lrEval_OpeningFunction gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to lrEval_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help lrEval

% Last Modified by GUIDE v2.5 05-Sep-2006 09:32:16

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @lrEval_OpeningFcn, ...
                   'gui_OutputFcn',  @lrEval_OutputFcn, ...
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


% --- Executes just before lrEval is made visible.
function lrEval_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to lrEval (see VARARGIN)

% Choose default command line output for lrEval
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes lrEval wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = lrEval_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in pb_refresh.
function pb_refresh_Callback(hObject, eventdata, handles)
% hObject    handle to pb_refresh (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


