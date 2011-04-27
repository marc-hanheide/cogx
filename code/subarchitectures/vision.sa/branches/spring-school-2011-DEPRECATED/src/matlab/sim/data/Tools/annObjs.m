function varargout = annObjs(varargin)
% ANNOBJS M-file for annObjs.fig
%      ANNOBJS, by itself, creates a new ANNOBJS or raises the existing
%      singleton*.
%
%      H = ANNOBJS returns the handle to a new ANNOBJS or the handle to
%      the existing singleton*.
%
%      ANNOBJS('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in ANNOBJS.M with the given input arguments.
%
%      ANNOBJS('Property','Value',...) creates a new ANNOBJS or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before annObjs_OpeningFunction gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to annObjs_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help annObjs

% Last Modified by GUIDE v2.5 04-Jul-2007 12:45:38

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @annObjs_OpeningFcn, ...
                   'gui_OutputFcn',  @annObjs_OutputFcn, ...
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


% --- Executes just before annObjs is made visible.
function annObjs_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to annObjs (see VARARGIN)

% Choose default command line output for annObjs
handles.output = hObject;

imgs=dir('img*');
img1st=imgs(1).name;
handles.II=str2num(img1st(4:6));

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes annObjs wait for user response (see UIRESUME)
% uiwait(handles.figure1);


%set(handles.ax_img,'Visible','off');

annShowImgs(handles);
annClear(handles);
annOpts(handles);




% --- Outputs from this function are returned to the command line.
function varargout = annObjs_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



function ed_ap_Callback(hObject, eventdata, handles)
% hObject    handle to ed_ap (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ed_ap as text
%        str2double(get(hObject,'String')) returns contents of ed_ap as a double
annDesc(handles);


% --- Executes during object creation, after setting all properties.
function ed_ap_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ed_ap (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function ed_typ_Callback(hObject, eventdata, handles)
% hObject    handle to ed_typ (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ed_typ as text
%        str2double(get(hObject,'String')) returns contents of ed_typ as a double
annDesc(handles);


% --- Executes during object creation, after setting all properties.
function ed_typ_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ed_typ (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function ed_mok_Callback(hObject, eventdata, handles)
% hObject    handle to ed_mok (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ed_mok as text
%        str2double(get(hObject,'String')) returns contents of ed_mok as a double


% --- Executes during object creation, after setting all properties.
function ed_mok_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ed_mok (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pb_prev.
function pb_prev_Callback(hObject, eventdata, handles)
% hObject    handle to pb_prev (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.II=handles.II-1;
guidata(hObject, handles);
annShowImgs(handles);
annClear(handles);


% --- Executes on button press in pb_next.
function pb_next_Callback(hObject, eventdata, handles)
% hObject    handle to pb_next (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

handles.II=handles.II+1;
guidata(hObject, handles);
annShowImgs(handles);
annClear(handles);


% --- Executes on button press in pb_load.
function pb_load_Callback(hObject, eventdata, handles)
% hObject    handle to pb_load (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
annLoad(handles,handles.II);

% --- Executes on button press in pb_loadPrev.
function pb_loadPrev_Callback(hObject, eventdata, handles)
% hObject    handle to pb_loadPrev (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
annLoad(handles,handles.II-1);


% --- Executes on button press in pb_clear.
function pb_clear_Callback(hObject, eventdata, handles)
% hObject    handle to pb_clear (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
annClear(handles);


% --- Executes on button press in pb_save.
function pb_save_Callback(hObject, eventdata, handles)
% hObject    handle to pb_save (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
annSave(handles);




function ed_no_Callback(hObject, eventdata, handles)
% hObject    handle to ed_no (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ed_no as text
%        str2double(get(hObject,'String')) returns contents of ed_no as a double
handles.II=str2double(get(handles.ed_no,'String'));
guidata(hObject, handles);
annShowImgs(handles);
annClear(handles);


% --- Executes during object creation, after setting all properties.
function ed_no_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ed_no (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end











function annOpts(handles)

global apNames;

apNames={
   'red';...1
   'green';...2
   'blue';...3
   'yellow';...4
   'white';...5
   'black';...6
   'gray';...7
   'metal';...8
   'orange';...9
   'purple';...10
   'pink';...11
   'brown';...12
   'tiny';...13
   'small';...14
   'large';...15
   'compact';...16
   'elongated';...17
   'cornered';...18
   'round';...19
   'long';...20
   'short'...21
};

apNames={
   'red';...1
   'green';...2
   'blue';...3
   'yellow';...4
   'small';...5
   'large';...6
   'square';...7
   'triangular';...8
   'circular';...9
   'rectangular';...10
};

desc=[];
for i=1:length(apNames);
   copt=[num2str(i,'%2d') '  ' cell2mat(apNames(i))];
   desc=[desc; cellstr(copt)];
end;
set(handles.lb_opt,'String',desc);

optt={'1 typical';'2 non-typical';'3 could be'};
set(handles.lb_optt,'String',optt);

optm={'1 mask OK';'2 maskI OK';'3 masks NOK'};
set(handles.lb_optm,'String',optm);


function annShowImgs(handles)

img=imread(['img' num2str(handles.II,'%03d') '.jpg']);
mask=imread(['msk' num2str(handles.II,'%03d') '.jpg']);
maskI=mask;%imread(['maskI' num2str(handles.II,'%03d') '.jpg']);

axes(handles.ax_img);
imshow(img);
axes(handles.ax_mask);
imshow(mask);
axes(handles.ax_maskI);
imshow(maskI);

set(handles.ed_no,'String',handles.II);



function annClear(handles)
set(handles.ed_ap,'String','');
set(handles.ed_typ,'String','');
set(handles.ed_mok,'String','');
set(handles.tx_desc,'String','');
set(handles.tx_typ,'String','');



function annDesc(handles);

global apNames;

ap=str2num(get(handles.ed_ap,'String'));
typ=str2num(get(handles.ed_typ,'String'));

desc=[];
for i=1:length(ap);
   desc=[desc;apNames(ap(i))];
end;

typs=[];
for i=1:length(typ);
   typs=[typs;typ(i)];
end;

set(handles.tx_desc,'String',desc);
set(handles.tx_typ,'String',typs);
   



function annSave(handles)

aps=str2num(get(handles.ed_ap,'String'));
typs=str2num(get(handles.ed_typ,'String'));
if length(aps)~=length(typs)
   typs=ones(1,length(aps));
end;   
gt.AP=[aps' typs'];
gt.MOK=str2num(get(handles.ed_mok,'String'));

fname=['dat' num2str(handles.II,'%03d')];
save(fname, 'gt');
disp([fname ' saved.']);
beep;


function annLoad(handles,JJ)
fname=['dat' num2str(JJ,'%03d')];
load(fname);
aps=gt.AP(:,1)';
typs=gt.AP(:,2)';
mok=gt.MOK;
set(handles.ed_ap,'String',num2str(aps));
set(handles.ed_typ,'String',num2str(typs));
set(handles.ed_mok,'String',num2str(mok));
annDesc(handles);


   



% --- Executes on selection change in lb_opt.
function lb_opt_Callback(hObject, eventdata, handles)
% hObject    handle to lb_opt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = get(hObject,'String') returns lb_opt contents as cell array
%        contents{get(hObject,'Value')} returns selected item from lb_opt
contents = get(hObject,'String');
sel=contents{get(hObject,'Value')};
newap=str2num(sel(1:2));
caps=str2num(get(handles.ed_ap,'String'));
aps=[caps newap];
set(handles.ed_ap,'String',num2str(aps));
annDesc(handles);


% --- Executes during object creation, after setting all properties.
function lb_opt_CreateFcn(hObject, eventdata, handles)
% hObject    handle to lb_opt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end




% --- Executes on selection change in lb_optt.
function lb_optt_Callback(hObject, eventdata, handles)
% hObject    handle to lb_optt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = get(hObject,'String') returns lb_optt contents as cell array
%        contents{get(hObject,'Value')} returns selected item from lb_optt
contents = get(hObject,'String');
sel=contents{get(hObject,'Value')};
newtyp=str2num(sel(1:2));
ctyps=str2num(get(handles.ed_typ,'String'));
typs=[ctyps newtyp];
set(handles.ed_typ,'String',num2str(typs));
annDesc(handles);


% --- Executes during object creation, after setting all properties.
function lb_optt_CreateFcn(hObject, eventdata, handles)
% hObject    handle to lb_optt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end




% --- Executes on selection change in lb_optm.
function lb_optm_Callback(hObject, eventdata, handles)
% hObject    handle to lb_optm (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = get(hObject,'String') returns lb_optm contents as cell array
%        contents{get(hObject,'Value')} returns selected item from lb_optm
contents = get(hObject,'String');
sel=contents{get(hObject,'Value')};
mok=str2num(sel(1:2));
set(handles.ed_mok,'String',num2str(mok));


% --- Executes during object creation, after setting all properties.
function lb_optm_CreateFcn(hObject, eventdata, handles)
% hObject    handle to lb_optm (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


