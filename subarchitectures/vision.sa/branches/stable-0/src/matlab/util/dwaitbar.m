function dwaitbar(frlen,msg);
%DWAITBAR  Create, display and close waitbar.
%   DWAITBAR(FRLEN,MSG) creates a waitbar if it has not been created yet, 
%   displays the waitbar of fractional length FRLEN (it has to be beteen 0 and 1),
%   and closes the waitbar if FRLEN equals 1. MSG is the title text (optional). 

global WBH;
if isempty(ishandle(WBH)) || ishandle(WBH)~=1
    if nargin<2 msg='Processing...'; end;
    WBH=waitbar(frlen,msg);
else
    waitbar(frlen,WBH);
end;
if frlen==1
    close(WBH);
end;

