%DINIT  Initialise default values.
%   DINIT initialises default values for all parameters required by DFIGURE.
%
%   See also DFIGURE, SWITCHDFIGURE, CLOSEFIGS, COMPOUNDFIGS.


global DFIGSW DFIGSH DFIGW DFIGH DFIGBW DFIGTBW DFIGAI DFIGAJ DFIGCI DFIGCJ DFIGON;

screen = get(0, 'ScreenSize');
DFIGSW = screen(3);
DFIGSH = screen(4);
if DFIGSW==800  %800x600
   DFIGW=123;
else    
   DFIGW=225;%185;
end;   
DFIGH=DFIGW;
DFIGBW=3;
DFIGTBW=22;
DFIGN=1;
DFIGON=1;
DFIGAI=1;
DFIGAJ=1;
DFIGCI=1;
DFIGCJ=1;

setDefFigPos(DFIGW);
colormap('GRAY');
cmap=colormap;
set(0, 'defaultfigurecolormap',cmap);  
set(0,'defaultfiguremenubar','none');

close(gcf);

dfigure(2,1,0);

set(0,'DefaultAxesLineStyleOrder',{'-',':','--'});
C=[0 0 1; 0 0.5 0; 1 0 0; 0 0.75 0.75; 0.75 0 0.75; 0.75 0.75 0; 
   0.25 0.25 0.25; 0 1 0; 0.5 0 0; 0 0 0.5];
set(0,'DefaultAxesColorOrder',C);

format compact;

global DCOLORS DCOLNUM;
DCOLORS=[0 0 1; 0 .5 0; 1 0 0; 0 .75 0; .75 .0 .75; .75 .75 0; .25 .25 .25; 0 1 0;.5 0 0; 0 0 .5];
DCOLNUM=1;

global DMARKS DMARKNUM;
DMARKS='+o*.xsd^v><ph';
DMARKNUM=1;

global WBH
