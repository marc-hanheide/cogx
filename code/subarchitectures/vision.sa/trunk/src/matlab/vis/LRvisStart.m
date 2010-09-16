function [LRguiL LRguiR]=LRvisStart


global LRguiLH LRguiR
global mC
global Coma
%Coma.Cnames=['Rd';'Gr';'Bl';'Yl';'Sm';'Lr';'Sq';'Cr';'Tr';'Rc'];
%Coma.Cnames=['Rd';'Gr';'Bl';'Yl';'Sm';'Lr'];
%Coma.Cnames=['Rd';'Gr';'Bl';'Yl';'Cm';'El'];
%Coma.avNames={'red';'green';'blue';'yellow';'compact';'elongated'};
Coma.Fnames=extAPfeatures;%['Hu';'Sa';'In';'Ar';'Df';'Ec'];

LRguiL=LRlearning;
LRguiLH=findobj(LRguiL, 'tag','pnModels');
showLmodels(mC,Coma.Fnames,Coma.Cnames,[2,4],LRguiLH);

global LRaxRoi LRtxFroi LRaxRec LRtxRec LRtxFrec LRtxFisRec LRtxFisRoi LRaxPts3d
LRguiR=LRrecognition;
LRaxRoi=findobj(LRguiR, 'Tag','ax_roi'); 
LRaxPts3d=findobj(LRguiR, 'Tag','lrax_pts3d');
LRtxFroi=findobj(LRguiR, 'tag','tx_Froi');
LRaxRec=findobj(LRguiR, 'Tag','ax_rec');
LRtxRec=findobj(LRguiR, 'Tag','tx_rec');
LRtxFrec=findobj(LRguiR, 'tag','tx_Frec');
LRtxFisRec=findobj(LRguiR, 'tag','tx_FisRec');
LRtxFisRoi=findobj(LRguiR, 'tag','tx_FisRoi');
% f1=figure;
% set(f1,'Visible','off');
% showROI;

