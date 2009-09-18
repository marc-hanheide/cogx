function lrH=LRevalStart

global LRevalH;

LRevalH.form=LReval;
LRevalH.ax_RS=findobj(LRevalH.form, 'tag','ax_RS');

set(LRevalH.ax_RS,'Box','on');

global Ft AVt NUs RSs;
global numVar varSiz varCol varNoise;

% [Xt,AVt]=genShapes(numVar,varSiz,varCol,varNoise);
% AVt=[[1:size(AVt,1)]' AVt];
% AVt=AVt(1:6,:);
% Bt=segmentImgs(Xt);
% Ft=extAPfeatures(Xt,Bt);
% %save testData Ft AVt
load testData


NUs=[];
RSs=[];

showEval(NUs,RSs,LRevalH.ax_RS);

lrH=LRevalH.form;