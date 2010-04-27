function varargout = setGvars(varargin)
% SETGVARS M-file for setGvars.fig
%      SETGVARS, by itself, creates a new SETGVARS or raises the existing
%      singleton*.
%
%      H = SETGVARS returns the handle to a new SETGVARS or the handle to
%      the existing singleton*.
%
%      SETGVARS('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in SETGVARS.M with the given input arguments.
%
%      SETGVARS('Property','Value',...) creates a new SETGVARS or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before setGvars_OpeningFunction gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to setGvars_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help setGvars

% Last Modified by GUIDE v2.5 30-Nov-2007 13:54:24

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
   'gui_Singleton',  gui_Singleton, ...
   'gui_OpeningFcn', @setGvars_OpeningFcn, ...
   'gui_OutputFcn',  @setGvars_OutputFcn, ...
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


% --- Executes just before setGvars is made visible.
function setGvars_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to setGvars (see VARARGIN)

% Choose default command line output for setGvars
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes setGvars wait for user response (see UIRESUME)
% uiwait(handles.figure1);

%init some vars

gvars={'loadData',1;'showRes',1;'N0ts',1;'N0ex',10;'THRs',num2str([2 4 6])};
handles.gvars=gvars;
guidata(hObject, handles);

set(handles.var1,'String',gvars{1,1});
set(handles.val1,'String',gvars{1,2});
set(handles.var2,'String',gvars{2,1});
set(handles.val2,'String',gvars{2,2});
set(handles.var3,'String',gvars{3,1});
set(handles.val3,'String',gvars{3,2});
set(handles.var4,'String',gvars{4,1});
set(handles.val4,'String',gvars{4,2});
set(handles.var5,'String',gvars{5,1});
set(handles.val5,'String',gvars{5,2});




% --- Outputs from this function are returned to the command line.
function varargout = setGvars_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;




function var1_Callback(hObject, eventdata, handles)
% hObject    handle to var1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of var1 as text
%        str2double(get(hObject,'String')) returns contents of var1 as a double


% --- Executes during object creation, after setting all properties.
function var1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to var1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
   set(hObject,'BackgroundColor','white');
end


function val1_Callback(hObject, eventdata, handles)
% hObject    handle to val1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of val1 as text
%        str2double(get(hObject,'String')) returns contents of val1 as a
%        double


% --- Executes during object creation, after setting all properties.
function val1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to val1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
   set(hObject,'BackgroundColor','white');
end



function var2_Callback(hObject, eventdata, handles)
% hObject    handle to var2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of var2 as text
%        str2double(get(hObject,'String')) returns contents of var2 as a double


% --- Executes during object creation, after setting all properties.
function var2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to var2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
   set(hObject,'BackgroundColor','white');
end



function val2_Callback(hObject, eventdata, handles)
% hObject    handle to val2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of val2 as text
%        str2double(get(hObject,'String')) returns contents of val2 as a double


% --- Executes during object creation, after setting all properties.
function val2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to val2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
   set(hObject,'BackgroundColor','white');
end



function var3_Callback(hObject, eventdata, handles)
% hObject    handle to var3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of var3 as text
%        str2double(get(hObject,'String')) returns contents of var3 as a double


% --- Executes during object creation, after setting all properties.
function var3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to var3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
   set(hObject,'BackgroundColor','white');
end



function val3_Callback(hObject, eventdata, handles)
% hObject    handle to val3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of val3 as text
%        str2double(get(hObject,'String')) returns contents of val3 as a double


% --- Executes during object creation, after setting all properties.
function val3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to val3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
   set(hObject,'BackgroundColor','white');
end



function var4_Callback(hObject, eventdata, handles)
% hObject    handle to var4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of var4 as text
%        str2double(get(hObject,'String')) returns contents of var4 as a double


% --- Executes during object creation, after setting all properties.
function var4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to var4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
   set(hObject,'BackgroundColor','white');
end



function val4_Callback(hObject, eventdata, handles)
% hObject    handle to val4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of val4 as text
%        str2double(get(hObject,'String')) returns contents of val4 as a double


% --- Executes during object creation, after setting all properties.
function val4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to val4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
   set(hObject,'BackgroundColor','white');
end



function var5_Callback(hObject, eventdata, handles)
% hObject    handle to var5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of var5 as text
%        str2double(get(hObject,'String')) returns contents of var5 as a double


% --- Executes during object creation, after setting all properties.
function var5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to var5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
   set(hObject,'BackgroundColor','white');
end



function val5_Callback(hObject, eventdata, handles)
% hObject    handle to val5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of val5 as text
%        str2double(get(hObject,'String')) returns contents of val5 as a double


% --- Executes during object creation, after setting all properties.
function val5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to val5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
   set(hObject,'BackgroundColor','white');
end


function var6_Callback(hObject, eventdata, handles)
% hObject    handle to var6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of var6 as text
%        str2double(get(hObject,'String')) returns contents of var6 as a double


% --- Executes during object creation, after setting all properties.
function var6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to var6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
   set(hObject,'BackgroundColor','white');
end



function val6_Callback(hObject, eventdata, handles)
% hObject    handle to val6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of val6 as text
%        str2double(get(hObject,'String')) returns contents of val6 as a double


% --- Executes during object creation, after setting all properties.
function val6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to val6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
   set(hObject,'BackgroundColor','white');
end



function bvar1_Callback(hObject, eventdata, handles)
% hObject    handle to bvar1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of bvar1 as text
%        str2double(get(hObject,'String')) returns contents of bvar1 as a double


% --- Executes during object creation, after setting all properties.
function bvar1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to bvar1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
   set(hObject,'BackgroundColor','white');
end



function bvar2_Callback(hObject, eventdata, handles)
% hObject    handle to bvar2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of bvar2 as text
%        str2double(get(hObject,'String')) returns contents of bvar2 as a double


% --- Executes during object creation, after setting all properties.
function bvar2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to bvar2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
   set(hObject,'BackgroundColor','white');
end



function bvar3_Callback(hObject, eventdata, handles)
% hObject    handle to bvar3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of bvar3 as text
%        str2double(get(hObject,'String')) returns contents of bvar3 as a double


% --- Executes during object creation, after setting all properties.
function bvar3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to bvar3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
   set(hObject,'BackgroundColor','white');
end



function bvar4_Callback(hObject, eventdata, handles)
% hObject    handle to bvar4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of bvar4 as text
%        str2double(get(hObject,'String')) returns contents of bvar4 as a double


% --- Executes during object creation, after setting all properties.
function bvar4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to bvar4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
   set(hObject,'BackgroundColor','white');
end



function bvar5_Callback(hObject, eventdata, handles)
% hObject    handle to bvar5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of bvar5 as text
%        str2double(get(hObject,'String')) returns contents of bvar5 as a double


% --- Executes during object creation, after setting all properties.
function bvar5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to bvar5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
   set(hObject,'BackgroundColor','white');
end



function bvar6_Callback(hObject, eventdata, handles)
% hObject    handle to bvar6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of bvar6 as text
%        str2double(get(hObject,'String')) returns contents of bvar6 as a double


% --- Executes during object creation, after setting all properties.
function bvar6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to bvar6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
   set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in bval1.
function bval1_Callback(hObject, eventdata, handles)
% hObject    handle to bval1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of bval1


% --- Executes on button press in bval2.
function bval2_Callback(hObject, eventdata, handles)
% hObject    handle to bval2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of bval2


% --- Executes on button press in bval3.
function bval3_Callback(hObject, eventdata, handles)
% hObject    handle to bval3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of bval3


% --- Executes on button press in bval4.
function bval4_Callback(hObject, eventdata, handles)
% hObject    handle to bval4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of bval4


% --- Executes on button press in bval5.
function bval5_Callback(hObject, eventdata, handles)
% hObject    handle to bval5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of bval5


% --- Executes on button press in bval6.
function bval6_Callback(hObject, eventdata, handles)
% hObject    handle to bval6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of bval6






% --- Executes on button press in pbGet.
function pbGet_Callback(hObject, eventdata, handles)
% hObject    handle to pbGet (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

gvars=refreshGvars(handles);
gvstr=gvars2str(gvars);
eval(['global ' gvstr]);

numgv=size(gvars,1);
for i=1:numgv
   gvars{i,2}=eval(gvars{i,1});
end;

handles.gvars=gvars;
guidata(hObject, handles);
refreshGvarsVals(handles);





% --- Executes on button press in pbSet.
function pbSet_Callback(hObject, eventdata, handles)
% hObject    handle to pbSet (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

gvars=refreshGvars(handles);
gvstr=gvars2str(gvars);
eval(['global ' gvstr]);

numgv=size(gvars,1);
for i=1:numgv
   vali=gvars{i,2};
   if size(vali,2)>1
      eval([gvars{i,1},'=[',num2str(vali),'];']);
   else
      eval([gvars{i,1},'=',num2str(vali),';']);
   end
end;

handles.gvars=gvars;
guidata(hObject, handles);




% --- Executes on button press in pbLoad.
function pbLoad_Callback(hObject, eventdata, handles)
% hObject    handle to pbLoad (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
load('gvars');
gvstr=gvars2str(gvars);
eval(['global ' gvstr]);

handles.gvars=gvars;
guidata(hObject, handles);

numgv=size(gvars,1);
for i=1:numgv
   vali=gvars{i,2}
   if size(vali,2)>1
      eval([gvars{i,1},'=[',num2str(vali),'];']);
      eval(['set(handles.val',num2str(i),',''String'',[',num2str(gvars{i,2}),']);']);
   else
      eval([gvars{i,1},'=',num2str(vali),';']);
      eval(['set(handles.val',num2str(i),',''String'',',num2str(gvars{i,2}),');']);
   end
end;

%refreshGvarsVals(handles);
disp('Gvars loaded.');



% --- Executes on button press in pbSave.
function pbSave_Callback(hObject, eventdata, handles)
% hObject    handle to pbSave (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

gvars=handles.gvars;
save('gvars','gvars');
disp('Gvars saved.');




% --- Executes on button press in bpDisplay.
function bpDisplay_Callback(hObject, eventdata, handles)
% hObject    handle to bpDisplay (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

handles.gvars

gvars=handles.gvars;
[gvstr,gvstrc]=gvars2str(gvars);
eval(['global ' gvstr]);
eval(gvstrc);




function [gvstr,gvstrc]=gvars2str(gvars)
numgv=size(gvars,1);
gvstr=[];
gvstrc=[];
for i=1:numgv
   gvstr=[gvstr gvars{i,1} ' '];
   gvstrc=[gvstrc gvars{i,1} ','];
end;
return



function gvars=refreshGvars(handles)
gvars={};
i=0;
for j=1:6
   i=i+1;
   varj=eval(['get(handles.var',num2str(j),',''String'');']);
   valj=eval(['str2num(get(handles.val',num2str(j),',''String''));']);
   if ~isempty(varj)
      gvars{i,1}=varj;
      gvars{i,2}=valj;
   end;
end;




function refreshGvarsVals(handles)
gvars=handles.gvars;
gvstr=gvars2str(gvars);
eval(['global ' gvstr]);
i=0;
for j=1:6
   i=i+1;
   varj=eval(['get(handles.var',num2str(j),',''String'');']);
   if ~isempty(varj)
%      eval(['set(handles.val',num2str(j),',''String'',',num2str(varj),');']);
      eval(['set(handles.val',num2str(j),',''String'',num2str(',varj,'));']);

   end
end;



