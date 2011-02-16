function varargout = DSdialogue(varargin)
% DSDIALOGUE M-file for DSdialogue.fig
%      DSDIALOGUE, by itself, creates a new DSDIALOGUE or raises the existing
%      singleton*.
%
%      H = DSDIALOGUE returns the handle to a new DSDIALOGUE or the handle to
%      the existing singleton*.
%
%      DSDIALOGUE('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in DSDIALOGUE.M with the given input arguments.
%
%      DSDIALOGUE('Property','Value',...) creates a new DSDIALOGUE or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before DSdialogue_OpeningFunction gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to DSdialogue_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help DSdialogue

% Last Modified by GUIDE v2.5 02-Sep-2008 23:09:53

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @DSdialogue_OpeningFcn, ...
                   'gui_OutputFcn',  @DSdialogue_OutputFcn, ...
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


% --- Executes just before DSdialogue is made visible.
function DSdialogue_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to DSdialogue (see VARARGIN)

% Choose default command line output for DSdialogue
handles.output = hObject;

%data coding words and sentences
handles.data=[];

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes DSdialogue wait for user response (see UIRESUME)
% uiwait(handles.figure1);
set(handles.lb_dlg, 'String', cell(0));
set(handles.lb_dlg, 'Value', 1);


% --- Outputs from this function are returned to the command line.
function varargout = DSdialogue_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in pb_req1.
function pb_req1_Callback(hObject, eventdata, handles)
% hObject    handle to pb_req1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.data=DSappendWord(handles.data,-1);guidata(hObject,handles);
sent=DScreateSentence(handles.data);
set(handles.tx_tutor,'String',sent);
set(handles.tx_robot,'String','');


% --- Executes on button press in pb_req2.
function pb_req2_Callback(hObject, eventdata, handles)
% hObject    handle to pb_req2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.data=DSappendWord(handles.data,-2);guidata(hObject,handles);
sent=DScreateSentence(handles.data);
set(handles.tx_tutor,'String',sent);
set(handles.tx_robot,'String','');


% --- Executes on button press in pb_req3.
function pb_req3_Callback(hObject, eventdata, handles)
% hObject    handle to pb_req3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.data=DSappendWord(handles.data,-3);guidata(hObject,handles);
sent=DScreateSentence(handles.data);
set(handles.tx_tutor,'String',sent);
set(handles.tx_robot,'String','');


% --- Executes on button press in pb_req4.
function pb_req4_Callback(hObject, eventdata, handles)
% hObject    handle to pb_req4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.data=DSappendWord(handles.data,-4);guidata(hObject,handles);
sent=DScreateSentence(handles.data);
set(handles.tx_tutor,'String',sent);
set(handles.tx_robot,'String','');


% --- Executes on button press in pb_req5.
function pb_req5_Callback(hObject, eventdata, handles)
% hObject    handle to pb_req5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.data=DSappendWord(handles.data,-5);guidata(hObject,handles);
sent=DScreateSentence(handles.data);
set(handles.tx_tutor,'String',sent);
set(handles.tx_robot,'String','');


% --- Executes on button press in pb_req6.
function pb_req6_Callback(hObject, eventdata, handles)
% hObject    handle to pb_req6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.data=DSappendWord(handles.data,-6);guidata(hObject,handles);
sent=DScreateSentence(handles.data);
set(handles.tx_tutor,'String',sent);
set(handles.tx_robot,'String','');


% --- Executes on button press in pb_req7.
function pb_req7_Callback(hObject, eventdata, handles)
% hObject    handle to pb_req7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.data=DSappendWord(handles.data,-7);guidata(hObject,handles);
sent=DScreateSentence(handles.data);
set(handles.tx_tutor,'String',sent);
set(handles.tx_robot,'String','');


% --- Executes on button press in pb_req15.
function pb_req15_Callback(hObject, eventdata, handles)
% hObject    handle to pb_req15 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.data=DSappendWord(handles.data,-15);guidata(hObject,handles);
sent=DScreateSentence(handles.data);
set(handles.tx_tutor,'String',sent);
set(handles.tx_robot,'String','');


% --- Executes on button press in pb_req16.
function pb_req16_Callback(hObject, eventdata, handles)
% hObject    handle to pb_req16 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.data=DSappendWord(handles.data,-16);guidata(hObject,handles);
sent=DScreateSentence(handles.data);
set(handles.tx_tutor,'String',sent);
set(handles.tx_robot,'String','');


% --- Executes on button press in pb_req17.
function pb_req17_Callback(hObject, eventdata, handles)
% hObject    handle to pb_req17 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.data=DSappendWord(handles.data,-17);guidata(hObject,handles);
sent=DScreateSentence(handles.data);
set(handles.tx_tutor,'String',sent);
set(handles.tx_robot,'String','');



% --- Executes on button press in pb_av1.
function pb_av1_Callback(hObject, eventdata, handles)
% hObject    handle to pb_av1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.data=DSappendWord(handles.data,1);guidata(hObject,handles);
sent=DScreateSentence(handles.data);
set(handles.tx_tutor,'String',sent);
set(handles.tx_robot,'String','');


% --- Executes on button press in pb_av2.
function pb_av2_Callback(hObject, eventdata, handles)
% hObject    handle to pb_av2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.data=DSappendWord(handles.data,2);guidata(hObject,handles);
sent=DScreateSentence(handles.data);
set(handles.tx_tutor,'String',sent);
set(handles.tx_robot,'String','');


% --- Executes on button press in pb_av3.
function pb_av3_Callback(hObject, eventdata, handles)
% hObject    handle to pb_av3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.data=DSappendWord(handles.data,3);guidata(hObject,handles);
sent=DScreateSentence(handles.data);
set(handles.tx_tutor,'String',sent);
set(handles.tx_robot,'String','');


% --- Executes on button press in pb_av4.
function pb_av4_Callback(hObject, eventdata, handles)
% hObject    handle to pb_av4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.data=DSappendWord(handles.data,4);guidata(hObject,handles);
sent=DScreateSentence(handles.data);
set(handles.tx_tutor,'String',sent);
set(handles.tx_robot,'String','');


% --- Executes on button press in pb_av5.
function pb_av5_Callback(hObject, eventdata, handles)
% hObject    handle to pb_av5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.data=DSappendWord(handles.data,5);guidata(hObject,handles);
sent=DScreateSentence(handles.data);
set(handles.tx_tutor,'String',sent);
set(handles.tx_robot,'String','');


% --- Executes on button press in pb_av6.
function pb_av6_Callback(hObject, eventdata, handles)
% hObject    handle to pb_av6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.data=DSappendWord(handles.data,6);guidata(hObject,handles);
sent=DScreateSentence(handles.data);
set(handles.tx_tutor,'String',sent);
set(handles.tx_robot,'String','');


% --- Executes on button press in pb_av7.
function pb_av7_Callback(hObject, eventdata, handles)
% hObject    handle to pb_av7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%handles.data=DSappendWord(handles.data,7);guidata(hObject,handles);
handles.data=DSappendWord(handles.data,5);guidata(hObject,handles);
sent=DScreateSentence(handles.data);
set(handles.tx_tutor,'String',sent);
set(handles.tx_robot,'String','');


% --- Executes on button press in pb_av8.
function pb_av8_Callback(hObject, eventdata, handles)
% hObject    handle to pb_av8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%handles.data=DSappendWord(handles.data,8);guidata(hObject,handles);
handles.data=DSappendWord(handles.data,6);guidata(hObject,handles);
sent=DScreateSentence(handles.data);
set(handles.tx_tutor,'String',sent);
set(handles.tx_robot,'String','');


% --- Executes on button press in pb_av9.
function pb_av9_Callback(hObject, eventdata, handles)
% hObject    handle to pb_av9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%handles.data=DSappendWord(handles.data,9);guidata(hObject,handles);
handles.data=DSappendWord(handles.data,7);guidata(hObject,handles);
sent=DScreateSentence(handles.data);
set(handles.tx_tutor,'String',sent);
set(handles.tx_robot,'String','');


% --- Executes on button press in pb_av10.
function pb_av10_Callback(hObject, eventdata, handles)
% hObject    handle to pb_av10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%handles.data=DSappendWord(handles.data,10);guidata(hObject,handles);
handles.data=DSappendWord(handles.data,8);guidata(hObject,handles);
sent=DScreateSentence(handles.data);
set(handles.tx_tutor,'String',sent);
set(handles.tx_robot,'String','');



% --- Executes on button press in pb_say.
function pb_say_Callback(hObject, eventdata, handles)
% hObject    handle to pb_say (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
DSextendDlg(['H: ' DScreateSentence(handles.data)]);
DSinterfaceOut(handles.data);
handles.data=[];guidata(hObject,handles);


% --- Executes on button press in pb_clear.
function pb_clear_Callback(hObject, eventdata, handles)
% hObject    handle to pb_clear (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.data=[];guidata(hObject,handles);
sent=[];
set(handles.tx_tutor,'String',sent);
set(handles.tx_robot,'String',sent);








% --- Executes on selection change in lb_dlg.
function lb_dlg_Callback(hObject, eventdata, handles)
% hObject    handle to lb_dlg (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = get(hObject,'String') returns lb_dlg contents as cell array
%        contents{get(hObject,'Value')} returns selected item from lb_dlg


% --- Executes during object creation, after setting all properties.
function lb_dlg_CreateFcn(hObject, eventdata, handles)
% hObject    handle to lb_dlg (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end




% --- Executes on selection change in listbox2.
function listbox2_Callback(hObject, eventdata, handles)
% hObject    handle to listbox2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = get(hObject,'String') returns listbox2 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from listbox2


% --- Executes during object creation, after setting all properties.
function listbox2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to listbox2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end




% --- Executes on button press in pb_gt.
function pb_gt_Callback(hObject, eventdata, handles)
% hObject    handle to pb_gt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global Data;

if ~isempty(Data.Cgt)
   handles.data=DSappendWord(handles.data,-7);guidata(hObject,handles);
   sent=DScreateSentence(handles.data);
   set(handles.tx_tutor,'String',sent);
   set(handles.tx_robot,'String','');

   cgt=find(Data.Cgt(:,Data.currImg))

   for i=1:length(cgt);
      handles.data=DSappendWord(handles.data,cgt(i));guidata(hObject,handles);
      sent=DScreateSentence(handles.data);
      set(handles.tx_tutor,'String',sent);
      set(handles.tx_robot,'String','');
   end
   DSextendDlg(['H: ' DScreateSentence(handles.data)]);
   DSinterfaceOut(handles.data);
   handles.data=[];guidata(hObject,handles);
end

