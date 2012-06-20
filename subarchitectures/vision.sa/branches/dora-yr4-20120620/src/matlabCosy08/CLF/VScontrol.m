function varargout = VScontrol(varargin)
% VSCONTROL M-file for VScontrol.fig
%      VSCONTROL, by itself, creates a new VSCONTROL or raises the existing
%      singleton*.
%
%      H = VSCONTROL returns the handle to a new VSCONTROL or the handle to
%      the existing singleton*.
%
%      VSCONTROL('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in VSCONTROL.M with the given input arguments.
%
%      VSCONTROL('Property','Value',...) creates a new VSCONTROL or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before VScontrol_OpeningFunction gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to VScontrol_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help VScontrol

% Last Modified by GUIDE v2.5 07-Nov-2007 00:18:17

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @VScontrol_OpeningFcn, ...
                   'gui_OutputFcn',  @VScontrol_OutputFcn, ...
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


% --- Executes just before VScontrol is made visible.
function VScontrol_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to VScontrol (see VARARGIN)

% Choose default command line output for VScontrol
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes VScontrol wait for user response (see UIRESUME)
% uiwait(handles.figure1);

axes(handles.axCurrImg);
axis off;

% set(handles.axCurrImg,'PlotBoxAspectRatioMode','auto', ...
%        'DataAspectRatioMode'   ,'auto', ...
%        'CameraViewAngleMode'   ,'auto');
set(handles.axCurrImg,'Visible','off');



% --- Outputs from this function are returned to the command line.
function varargout = VScontrol_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in pbNext.
function pbNext_Callback(hObject, eventdata, handles)
% hObject    handle to pbNext (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

VSnextImg;
