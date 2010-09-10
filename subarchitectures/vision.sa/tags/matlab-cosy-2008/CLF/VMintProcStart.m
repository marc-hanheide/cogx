function vmipH=VMintProcStart

global avNames lbipH;

%avNames={'red';'green';'blue';'yellow';'small';'large';'square';'circular';'triangular';'rectangular'};
%avNames={'red';'green';'blue';'yellow';'compact';'elongated'};%;'square';'circular';'triangular';'rectangular'};

vmipH=VMintProc;

lbipH=findobj(vmipH, 'tag','lb_intProc');


