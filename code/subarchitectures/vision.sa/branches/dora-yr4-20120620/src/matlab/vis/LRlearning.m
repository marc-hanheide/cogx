function varargout = LRlearning(varargin)
% LRLEARNING M-file for LRlearning.fig
%      LRLEARNING, by itself, creates a new LRLEARNING or raises the existing
%      singleton*.
%
%      H = LRLEARNING returns the handle to a new LRLEARNING or the handle to
%      the existing singleton*.
%
%      LRLEARNING('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in LRLEARNING.M with the given input arguments.
%
%      LRLEARNING('Property','Value',...) creates a new LRLEARNING or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before LRlearning_OpeningFunction gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to LRlearning_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help LRlearning

% Last Modified by GUIDE v2.5 24-Oct-2009 14:59:17

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @LRlearning_OpeningFcn, ...
                   'gui_OutputFcn',  @LRlearning_OutputFcn, ...
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


% --- Executes just before LRlearning is made visible.
function LRlearning_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to LRlearning (see VARARGIN)

% Choose default command line output for LRlearning
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes LRlearning wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = LRlearning_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in pbOptions.
function pbOptions_Callback(hObject, eventdata, handles)
% hObject    handle to pbOptions (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
LRcontrol




% --- Executes on button press in pb_closeModels.
function pb_closeModels_Callback(hObject, eventdata, handles)
% hObject    handle to pb_closeModels (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

lastfig=figure;
for i=1:lastfig
   if ishandle(i)
      figname=get(i,'Name');
      if length(figname)>=2 && isequal(figname(1:2),'mC')
         close(i);
      end
   end
end;
close(lastfig);




% --- Executes on button press in pb_redrawModels.
function pb_redrawModels_Callback(hObject, eventdata, handles)
% hObject    handle to pb_redrawModels (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
LRvisUpdate

