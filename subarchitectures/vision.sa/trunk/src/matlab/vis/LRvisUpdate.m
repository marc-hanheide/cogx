function LRvisUpdate

global Coma

%Coma.Cnames=['Rd';'Gr';'Bl';'Yl';'Sm';'Lr';'Sq';'Cr';'Tr';'Rc'];
%Coma.Cnames=['Rd';'Gr';'Bl';'Yl';'Sm';'Lr'];
%Coma.Cnames=['Rd';'Gr';'Bl';'Yl';'Cm';'El'];
%Coma.avNames={'red';'green';'blue';'yellow';'compact';'elongated'};
%Fnames=extAPfeatures;%['Hu';'Sa';'In';'Ar';'Cp';'Ec'];

global mC LRguiLH
showLmodels(mC,Coma.Fnames,Coma.Cnames,[2 5],LRguiLH);
