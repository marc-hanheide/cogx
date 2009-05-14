function LRguiL=LRvisStart


global LRguiLH mAV;
global avAcronyms avNames
%avAcronyms=['Rd';'Gr';'Bl';'Yl';'Sm';'Lr';'Sq';'Cr';'Tr';'Rc'];
%avAcronyms=['Rd';'Gr';'Bl';'Yl';'Sm';'Lr'];
%avAcronyms=['Rd';'Gr';'Bl';'Yl';'Cm';'El'];
%avNames={'red';'green';'blue';'yellow';'compact';'elongated'};
Fnames=extAPfeatures;%['Hu';'Sa';'In';'Ar';'Df';'Ec'];

LRguiL=LRlearning;
LRguiLH=findobj(LRguiL, 'tag','pnModels');
showLmodels(mAV,Fnames,avAcronyms,[2,4],LRguiLH);


% 
% 
% global LRvisH;
% 
% %avAcronyms=['Rd';'Gr';'Bl';'Yl';'Sm';'Lr';'Sq';'Cr';'Tr';'Rc'];
% %avAcronyms=['Rd';'Gr';'Bl';'Yl';'Sm';'Lr'];
% avAcronyms=['Rd';'Gr';'Bl';'Yl';'Cm';'El'];
% avNames={'red';'green';'blue';'yellow';'compact';'elongated'};
% Fnames=extAPfeatures;%['Hu';'Sa';'In';'Ar';'Df';'Ec'];
% 
% 
% 
% % FigAss=dfigure(4,2,'LR Visualisation - Associations');
% % resizeFigs(gcf,2,1);
% 
% LRvisH.form=LRvis;
% 
% LRvisH.ax_assoc=findobj(LRvisH.form, 'tag','ax_assoc');
% %LRvisH.ax_assoc=get(LRvisH.form,'CurrentAxes');
% LRvisH.ax_models=findobj(LRvisH.form, 'tag','ax_models');
% %LRvisH.ax_models=get(LRvisH.form,'CurrentAxes');
% 
% global mAV;
% showAssoc(mAV,Fnames,avAcronyms,LRvisH.ax_assoc)
% %showCmodels(mAV,Fnames,avAcronyms,LRvisH.ax_models)
% 
% lrH=LRvisH.form;
% 
% global LRvisMH;
% LRvisMH=figure;
% set(LRvisMH,'NumberTitle','off');
% set(LRvisMH,'Name','LRvisModels');
% %resizeFigs(LRvisMH,2,1.5);
% set(LRvisMH,'Position',[304   808   601   361]);
% set(LRvisMH,'MenuBar','none');
% showCmodels(mAV,Fnames,avAcronyms,LRvisMH);
% 
