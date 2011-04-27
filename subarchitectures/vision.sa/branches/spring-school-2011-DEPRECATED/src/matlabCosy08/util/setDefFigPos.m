function setDefFigPos(varargin)
%SETDEFFIGPOS  Set default figure position and size.
%   SETDEFFIGPOS(W) Sets the default figure size to WxW and the default position 
%   to the right upper corner of the screen.
%
%   SETDEFFIGPOS(W,H) Sets the default figure size to WxH.
%
%   SETDEFFIGPOS(X,Y,W) Sets the default figure size to WxW and the default 
%   figure position to (X,Y).
% 
%   SETDEFFIGPOS(X,Y,W,H) Sets the default figure size to WxH and the default 
%   figure position to (X,Y).
%
%   See also DINIT, DFIGURE, CLOSEFIGS, COMPOUNDFIGS.

global DFIGW DFIGH DFIGBW DFIGTBW;

screen = get(0, 'ScreenSize');
swidth = screen(3);
sheight = screen(4);

nin=nargin;
switch nin
	case 1 %setDefFigPos(w)
     iwidth=varargin{1};
     iheight=iwidth;
     ix=swidth-iwidth-DFIGBW;
     iy=sheight-iheight-DFIGTBW;
  case 2 %setDefFigPos(w,h)
     iwidth=varargin{1};
     iheight=varargin{2};
     ix=swidth-iwidth-DFIGBW;
     iy=sheight-iheight-DFIGTBW;
  case 3 %setDefFigPos(x,y,w)
     ix=varargin{1};
     iy=varargin{2};
     iwidth=varargin{3};
     iheight=iwidth;
  case 4 %setDefFigPos(x,y,w,h)
     ix=varargin{1};
     iy=varargin{2};
     iwidth=varargin{3};
     iheight=varargin{4};
end

set(0, 'defaultfigureposition',[ix iy iwidth iheight]);  
  
  