function varargout = LRrecAtt(varargin)
% LRRECATT M-file for LRrecAtt.axfig
%      LRRECATT, by itself, creates a new LRRECATT or raises the existing
%      singleton*.
%
%      H = LRRECATT returns the handle to a new LRRECATT or the handle to
%      the existing singleton*.
%
%      LRRECATT('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in LRRECATT.M with the given input arguments.
%
%      LRRECATT('Property','Value',...) creates a new LRRECATT or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before LRrecAtt_OpeningFunction gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to LRrecAtt_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help LRrecAtt

% Last Modified by GUIDE v2.5 05-May-2008 10:37:31

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @LRrecAtt_OpeningFcn, ...
                   'gui_OutputFcn',  @LRrecAtt_OutputFcn, ...
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


% --- Executes just before LRrecAtt is made visible.
function LRrecAtt_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to LRrecAtt (see VARARGIN)

% Choose default command line output for LRrecAtt
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes LRrecAtt wait for user response (see UIRESUME)
% uiwait(handles.figure1);


set(handles.axfig,'Visible','off');
set(handles.axprob,'Visible','off');


% --- Outputs from this function are returned to the command line.
function varargout = LRrecAtt_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;
