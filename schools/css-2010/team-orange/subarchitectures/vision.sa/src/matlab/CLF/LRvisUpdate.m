function LRvisUpdate

global avNames avAcronyms

%avAcronyms=['Rd';'Gr';'Bl';'Yl';'Sm';'Lr';'Sq';'Cr';'Tr';'Rc'];
%avAcronyms=['Rd';'Gr';'Bl';'Yl';'Sm';'Lr'];
%avAcronyms=['Rd';'Gr';'Bl';'Yl';'Cm';'El'];
%avNames={'red';'green';'blue';'yellow';'compact';'elongated'};
Fnames=extAPfeatures;%['Hu';'Sa';'In';'Ar';'Cp';'Ec'];

global mAV LRguiLH
showLmodels(mAV,Fnames,avAcronyms,[2 4],LRguiLH);
