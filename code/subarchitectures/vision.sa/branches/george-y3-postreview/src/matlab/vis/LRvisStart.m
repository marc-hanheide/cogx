function LRvisStart


global Figs 
global mC
global Coma
%Coma.Cnames=['Rd';'Gr';'Bl';'Yl';'Sm';'Lr';'Sq';'Cr';'Tr';'Rc'];
%Coma.Cnames=['Rd';'Gr';'Bl';'Yl';'Sm';'Lr'];
%Coma.Cnames=['Rd';'Gr';'Bl';'Yl';'Cm';'El'];
%Coma.avNames={'red';'green';'blue';'yellow';'compact';'elongated'};
Coma.Fnames=extAPfeatures;%['Hu';'Sa';'In';'Ar';'Df';'Ec'];

Figs.LRguiL.main=LRlearning;
Figs.LRguiL.pnModels=findobj(Figs.LRguiL.main, 'tag','pnModels');
Figs.LRguiL.tx_Fb=findobj(Figs.LRguiL.main, 'tag','tx_Fb');
showLmodels(mC,Coma.Fnames,Coma.Cnames,[2,4],Figs.LRguiL);

%global LRaxRoi LRtxFroi LRaxRec LRtxRec LRtxFrec LRtxFisRec LRtxFisRoi LRaxPts3d
LRguiR=LRrecognition; 
Figs.LRguiR.main=LRguiR;
Figs.LRguiR.LRaxRoi=findobj(LRguiR, 'Tag','ax_roi'); 
Figs.LRguiR.LRaxPts3d=findobj(LRguiR, 'Tag','lrax_pts3d');
Figs.LRguiR.LRtxFroi=findobj(LRguiR, 'tag','tx_Froi');
Figs.LRguiR.LRaxRec=findobj(LRguiR, 'Tag','ax_rec');
Figs.LRguiR.LRtxRec=findobj(LRguiR, 'Tag','tx_rec');
Figs.LRguiR.LRtxFrec=findobj(LRguiR, 'tag','tx_Frec');
Figs.LRguiR.LRtxFisRec=findobj(LRguiR, 'tag','tx_FisRec');
Figs.LRguiR.LRtxFisRoi=findobj(LRguiR, 'tag','tx_FisRoi');
% f1=figure;
% set(f1,'Visible','off');
% showROI;

