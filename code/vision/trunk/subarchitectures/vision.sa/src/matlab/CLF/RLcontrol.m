function varargout = RLcontrol(varargin)
% RLCONTROL M-file for RLcontrol.fig
%      RLCONTROL, by itself, creates a new RLCONTROL or raises the existing
%      singleton*.
%
%      H = RLCONTROL returns the handle to a new RLCONTROL or the handle to
%      the existing singleton*.
%
%      RLCONTROL('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in RLCONTROL.M with the given input arguments.
%
%      RLCONTROL('Property','Value',...) creates a new RLCONTROL or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before RLcontrol_OpeningFunction gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to RLcontrol_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help RLcontrol

% Last Modified by GUIDE v2.5 09-Nov-2007 19:02:16

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @RLcontrol_OpeningFcn, ...
                   'gui_OutputFcn',  @RLcontrol_OutputFcn, ...
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


% --- Executes just before RLcontrol is made visible.
function RLcontrol_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to RLcontrol (see VARARGIN)

% Choose default command line output for RLcontrol
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes RLcontrol wait for user response (see UIRESUME)
% uiwait(handles.figure1);

global mR mDR mRFS RLvisH RLvisMH
[mR,mDR,mRFS]=KDBFinit;

RLvisH.form=RLvis;
RLvisH.ax_assoc=findobj(RLvisH.form, 'tag','ax_assoc');

RLvisMH=figure;
set(RLvisMH,'NumberTitle','off');
set(RLvisMH,'Name','RLvisModels');
set(RLvisMH,'Position',[304   808   601   361]);
set(RLvisMH,'MenuBar','none');
refreshRmodels;



% --- Outputs from this function are returned to the command line.
function varargout = RLcontrol_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in pb_rel1.
function pb_rel1_Callback(hObject, eventdata, handles)
% hObject    handle to pb_rel1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[pa,pb]=genRel(1);
showAB(pa,pb,handles);
updateRels(pa,pb,1);
refreshRassoc;
refreshRmodels;

% --- Executes on button press in pb_rel2.
function pb_rel2_Callback(hObject, eventdata, handles)
% hObject    handle to pb_rel2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[pa,pb]=genRel(2);
showAB(pa,pb,handles);
updateRels(pa,pb,2);
refreshRassoc;
refreshRmodels;

% --- Executes on button press in pb_rel3.
function pb_rel3_Callback(hObject, eventdata, handles)
% hObject    handle to pb_rel3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[pa,pb]=genRel(3);
showAB(pa,pb,handles);
updateRels(pa,pb,3);
refreshRassoc;
refreshRmodels;


% --- Executes on button press in pb_rel4.
function pb_rel4_Callback(hObject, eventdata, handles)
% hObject    handle to pb_rel4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[pa,pb]=genRel(4);
showAB(pa,pb,handles);
updateRels(pa,pb,4);
refreshRassoc;
refreshRmodels;


% --- Executes on button press in pb_rel5.
function pb_rel5_Callback(hObject, eventdata, handles)
% hObject    handle to pb_rel5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[pa,pb]=genRel(5);
showAB(pa,pb,handles);
updateRels(pa,pb,5);
refreshRassoc;
refreshRmodels;


% --- Executes on button press in pb_rel6.
function pb_rel6_Callback(hObject, eventdata, handles)
% hObject    handle to pb_rel6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[pa,pb]=genRel(6);
showAB(pa,pb,handles);
updateRels(pa,pb,6);
refreshRassoc;
refreshRmodels;


% --- Executes on button press in pb_load.
function pb_load_Callback(hObject, eventdata, handles)
% hObject    handle to pb_load (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global mR mDR mRFS;

[FileName,PathName] = uigetfile('*.mat','Select the file containing R models');
if FileName~=0
   fname=[PathName FileName];
   load(fname);

   disp(['R models loaded from ' fname ' .']);
   [nlc,nec,LC,EC,confs]=numConcepts(mR);
   fprintf('nlc=%d nec=%d   confs=', nlc, nec);disp(confs);

   refreshRassoc;
   refreshRmodels;
   
end;


% --- Executes on button press in pb_save.
function pb_save_Callback(hObject, eventdata, handles)
% hObject    handle to pb_save (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global mR mDR mRFS;

[FileName,PathName] = uiputfile('*.mat','Save file name','mRs.mat');
if FileName~=0
   fname=[PathName FileName];
   save(fname,'mR','mRFS','mDR');
   disp(['R models saved in ' fname ' .']);
   [nlc,nec,LC,EC,confs]=numConcepts(mR);
   fprintf('nlc=%d nec=%d   confs=', nlc, nec);disp(confs);
end;






function showAB(pa,pb,handles)

cla(handles.ax_rels);
text(pa(1),pa(2), 'A', 'Color','b','Parent',handles.ax_rels);
text(pb(1),pb(2), 'B', 'Color','r','Parent',handles.ax_rels);

axis(handles.ax_rels, [0 640 0 480]);
%set(handles.ax_rels,'YDir','reverse')
set(handles.ax_rels,'XTick',100:100:600);
set(handles.ax_rels,'YTick',100:100:400);
set(handles.ax_rels,'XTickLabel',[]);
set(handles.ax_rels,'YTickLabel',[]);



function updateRels(pa,pb,relNum,handles)
 
global mR mDR mRFS 
f=extRelFeatures([pa pb]);
[mR,mDR,mRFS]=KDBFupdate(f(:,1),relNum,mR,mDR,mRFS);



function refreshRassoc

global mR mDR mRFS RLvisH;

RAcronyms={'TL';'TR';'CT';'FT';'NT';'FF'};
Fnames=['x ';'y ';'dx';'dy';'d '];

ax=RLvisH.ax_assoc;


NPX=100;
NPY=4;
numF=size(Fnames,1);
numR=size(RAcronyms,1);

cla(ax);
set(ax,'Xlim',[0 NPX]);
set(ax,'Ylim',[0 NPY]);
set(ax,'PlotBoxAspectRatioMode','auto', ...
   'DataAspectRatioMode'   ,'auto', ...
   'CameraViewAngleMode'   ,'auto');
set(ax,'Visible','off');

pxF=zeros(1,numF);
pxR=zeros(1,numR);
for i=1:numF
   pxF(i)=(i-1)*NPX/numF;
   text(pxF(i),4,Fnames(i,:), 'Parent',ax);
end;
for i=1:numR
   pxR(i)=(i-1)*NPX/numR;
   text(pxR(i),1,RAcronyms(i,:), 'Parent',ax);
end;

if ~isempty(mR)

   numcR=size(mR,2);
   names=[mR.name];
   Fbs=[mR.Fb];
   confs=[mR.conf];

   for i=1:numcR
      if mR(i).conf>1
         line([pxR(names(i))+2,pxF(Fbs(i))+2],[1.2,3.8], 'Parent',ax);
      end
   end

   for i=1:numcR
      if mR(i).conf>1
         text(pxR(names(i))-1,0,num2str(confs(i),'%02.2f'), 'Color','blue','FontSize',8,'Parent',ax);
      end;
   end;

end



function refreshRmodels

global mR mDR mRFS RLvisMH

Cnames={'TL';'TR';'CT';'FT';'NT';'FF'};
Fnames=['x ';'y ';'dx';'dy';'d '];

ax=RLvisMH;
sphw=[2 3];
numC=length(mR);

figure(ax);
clf(ax);

for i=1:numC
   if ~isempty(mR(1).name)
      subplot(sphw(1),sphw(2),i)
      showDecomposedPdf(mR(i));
      %set(gca,'FontSize',16);
      title([Cnames(mR(i).name,:)]);
      xlabel(Fnames(mR(i).Fb,:));
      set(gca,'ytick',[]);
      axis tight;
      
      alim=axis;
      xticks=[alim(1),(alim(1)+alim(2))/2,alim(2)];
      set(gca,'XTick',xticks);
      set(gca,'XTickLabel',{num2str(xticks(1),'%3.f'),num2str(xticks(2),'%3.f'),num2str(xticks(3),'%3.f')})    
      
   end
end;


