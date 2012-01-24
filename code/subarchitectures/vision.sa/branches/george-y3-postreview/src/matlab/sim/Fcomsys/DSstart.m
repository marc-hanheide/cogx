function DSstart

global senMap Figs;% txrH lbdH;

senMap={...
   'What do you see?', '';...//1
   'Is it ', '?';...//2
   'It is ', '.';...//3
   'It is also ', '.';...//4
   'Yes, it is.', '';...//5
   'No, it is not.', '';...//6
   'This is a ',  ' object.';...//7
   'I see a ',  ' object.';...//8
   'I don''t know this object.', '';...//9
   'I think it is ',  '.';...//10
   'I think it is also ',  '.';...//11
   'OK.', '';...;//12
   'Is it also ', '?';...//13
   'I think I see a ', ' object.';...//14
   'Yes, that''s correct.', '';...//15
%   'No, it is ', '.';...//16
   'This is not a ', ' object.';...//16
   'No, it is not ', '.';...//17
   'Is this object ', '?';...//18
   'What is it like?', '';...//19
};

dlgH=DSdialogue;
Figs.dlgH.main=dlgH;

Figs.dlgH.txrH=findobj(dlgH, 'tag','tx_robot');
Figs.dlgH.lbdH=findobj(dlgH, 'tag','lb_dlg');

