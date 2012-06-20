function varargout = ATcontrol(varargin)
% ATCONTROL M-file for ATcontrol.fig
%      ATCONTROL, by itself, creates a new ATCONTROL or raises the existing
%      singleton*.
%
%      H = ATCONTROL returns the handle to a new ATCONTROL or the handle to
%      the existing singleton*.
%
%      ATCONTROL('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in ATCONTROL.M with the given input arguments.
%
%      ATCONTROL('Property','Value',...) creates a new ATCONTROL or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before ATcontrol_OpeningFunction gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to ATcontrol_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help ATcontrol

% Last Modified by GUIDE v2.5 01-Sep-2006 17:48:17

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @ATcontrol_OpeningFcn, ...
                   'gui_OutputFcn',  @ATcontrol_OutputFcn, ...
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


% --- Executes just before ATcontrol is made visible.
function ATcontrol_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to ATcontrol (see VARARGIN)

% Choose default command line output for ATcontrol
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes ATcontrol wait for user response (see UIRESUME)
% uiwait(handles.figure1);

set(handles.bg_att,'SelectionChangeFcn',@bg_att_SelectionChangeFcn);


% --- Outputs from this function are returned to the command line.
function varargout = ATcontrol_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;





% --- Executes on button press in radiobutton5.
function bg_att_SelectionChangeFcn(hObject, eventdata, handles)
% hObject    handle to radiobutton5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of radiobutton5
beep;
attOnOff;



