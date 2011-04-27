function figno=dfigure(varargin);
%function [figno,name]=dfigure(varargin);
%DFIGURE  Create figure window.
%   DFIGURE creates a new figure window at incremented position (DFIGCI,DFIGCJ).
%   DFIGURE(TITLE) creates a new titled figure window.
%   DFIGURE(AI) creates a new figure window at position (AI,DFIGCJ).
%   DFIGURE(AI,TITLE) creates a new titled figure window at position (AI,DFIGCJ).
%   DFIGURE(AI,AJ) creates a new figure window at position (AI,AJ).
%   DFIGURE(AI,AJ,TITLE) creates a new titled figure window at position (AI,AJ).
%
%   If input parameter AI==0 a new figure window is not created, only the current
%   values (DFIGCI,DFIGCJ) are incremented. If all three paramethers are given and
%   the third parameter is 0 the next dfigure will create a new figure at position 
%   (AI,AJ). If the second parameter is 0 the DFIGCJ is incremented by 1.
%
%   If DFIGON is set to 0 DFIGURE does nothing.
%
%   Position (0,0) is in the right upper corner of the screen, an unit is
%   a width/height of a figure window.
%
%   All default values for parameters DFIGCI, DFIGCJ, etc. are set by DINIT.
%   DINIT has to be executed before first execution of DFIGURE.
%
%   See also DINIT, SWITCHDFIGURE, CLOSEFIGS, COMPOUNDFIGS.

global DFIGSW DFIGSH DFIGW DFIGH DFIGBW DFIGTBW DFIGAI DFIGAJ DFIGCI DFIGCJ DFIGON;

if DFIGON==0 return; end;

figai=-1;
figcj=1;
name='';
figno=0;

nin=nargin;
switch nin
   case 0
   case 1
      if isa(varargin{1},'char')
         name=varargin{1};
      else
         figai=varargin{1};
      end;
   case 2
      if isa(varargin{2},'char')
         name=varargin{2};
         figai=varargin{1};
      else
         figai=varargin{1};
         figcj=varargin{2};
      end;
   case 3
      figai=varargin{1};
      figcj=varargin{2};
      name=varargin{3};
end;

if figai>0
   DFIGAI=figai;
   if figcj>0
      DFIGAJ=floor(DFIGSH/(DFIGH+DFIGBW+DFIGTBW));
      DFIGCI=1;
      DFIGCJ=figcj;
   else
%       DFIGCJ=DFIGCJ+1;
%       if DFIGCJ>DFIGAJ
%          DFIGCJ=1;
%       end;
   end
else
   DFIGCI=DFIGCI+1;
   if DFIGCI>DFIGAI
      DFIGCI=1;
      DFIGCJ=DFIGCJ+1;
      if DFIGCJ>DFIGAJ
         DFIGCJ=1;
      end;
   end;
end;

if ~isempty(name) & name==0
   if figai>0
      DFIGCI=figai;
   else   
      DFIGCI=DFIGCI-1;
   end   
   figai=0;
end;

if figai~=0
   x1=DFIGSW-(DFIGW+2*DFIGBW+2)*(DFIGAI-DFIGCI+1)+5;
   y1=DFIGSH-(DFIGH+DFIGTBW+DFIGBW+2)*DFIGCJ+5;
   figno=figure('Position', [x1, y1, DFIGW, DFIGH]);
   if ~isempty(name)
      set(figno,'NumberTitle','off');
      name=['f',num2str(figno),': ',name];
      set(figno,'Name',name);
   end;
   set(gca,'PlotBoxAspectRatioMode','manual');
   set(gca,'DataAspectRatioMode','manual');
   set(gca,'CameraViewAngleMode','manual');
end;

